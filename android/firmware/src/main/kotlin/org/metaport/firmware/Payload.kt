package org.metaport.firmware

import org.apache.commons.compress.compressors.bzip2.BZip2CompressorInputStream
import org.tukaani.xz.XZInputStream
import java.io.ByteArrayInputStream
import java.io.File
import java.io.InputStream
import java.io.RandomAccessFile
import java.security.MessageDigest

/**
 * Reader for Android A/B OTA payloads (`payload.bin`, update_engine "CrAU" format), which is
 * how Horizon OS updates are packaged. Only *full* payloads can be extracted: incremental ones
 * patch the previous partition contents, which the user would also have to supply.
 *
 * Nothing here decrypts or bypasses anything: full payloads store partition data in the clear
 * (raw/xz/bz2), and every operation and partition is verified against the hashes in the
 * manifest.
 */
class Payload private constructor(
    private val file: File,
    val formatVersion: Long,
    val blockSize: Int,
    val minorVersion: Int,
    val partitions: List<Partition>,
    private val dataOffset: Long,
) {
    enum class OpType(val id: Int) {
        REPLACE(0), REPLACE_BZ(1), MOVE(2), BSDIFF(3), SOURCE_COPY(4), SOURCE_BSDIFF(5), ZERO(6), DISCARD(7),
        REPLACE_XZ(8), PUFFDIFF(9), BROTLI_BSDIFF(10), ZUCCHINI(11), LZ4DIFF_BSDIFF(12), LZ4DIFF_PUFFDIFF(13), ZSTD(14);

        val isFullOp get() = this == REPLACE || this == REPLACE_BZ || this == REPLACE_XZ || this == ZERO || this == DISCARD

        companion object {
            fun of(id: Int) = entries.firstOrNull { it.id == id } ?: throw FirmwareFormatException("unknown operation $id")
        }
    }

    data class Extent(val startBlock: Long, val numBlocks: Long)

    data class Operation(
        val type: OpType,
        val dataOffset: Long,
        val dataLength: Long,
        val dstExtents: List<Extent>,
        val dataSha256: ByteArray?,
    )

    data class Partition(val name: String, val size: Long, val sha256: ByteArray?, val operations: List<Operation>) {
        val isFull get() = operations.all { it.type.isFullOp }
    }

    val isFull get() = minorVersion == 0 && partitions.all { it.isFull }

    /**
     * Writes one partition image to [out]. Throws on any hash mismatch. [progress] receives the
     * fraction of operations done.
     */
    fun extract(name: String, out: File, progress: (Float) -> Unit = {}) {
        val p = partitions.firstOrNull { it.name == name } ?: throw IllegalArgumentException("no partition $name")
        if (!p.isFull) throw UnsupportedOperationException("partition $name is incremental (needs the previous firmware)")
        RandomAccessFile(file, "r").use { src ->
            RandomAccessFile(out, "rw").use { dst ->
                dst.setLength(p.size)
                p.operations.forEachIndexed { i, op ->
                    applyOperation(src, dst, op)
                    progress((i + 1).toFloat() / p.operations.size)
                }
            }
        }
        if (p.sha256 != null) {
            val actual = out.inputStream().use(::sha256)
            if (!actual.contentEquals(p.sha256)) throw FirmwareFormatException("partition $name hash mismatch")
        }
    }

    private fun applyOperation(src: RandomAccessFile, dst: RandomAccessFile, op: Operation) {
        val blockBytes = blockSize.toLong()
        val targetBytes = op.dstExtents.sumOf { it.numBlocks } * blockBytes
        val data: InputStream = when (op.type) {
            OpType.ZERO, OpType.DISCARD -> ZeroStream(targetBytes)
            else -> {
                val raw = ByteArray(op.dataLength.toInt())
                src.seek(dataOffset + op.dataOffset)
                src.readFully(raw)
                if (op.dataSha256 != null && !sha256(ByteArrayInputStream(raw)).contentEquals(op.dataSha256)) {
                    throw FirmwareFormatException("operation data hash mismatch at ${op.dataOffset}")
                }
                when (op.type) {
                    OpType.REPLACE -> ByteArrayInputStream(raw)
                    OpType.REPLACE_XZ -> XZInputStream(ByteArrayInputStream(raw))
                    OpType.REPLACE_BZ -> BZip2CompressorInputStream(ByteArrayInputStream(raw))
                    else -> throw UnsupportedOperationException("operation ${op.type}")
                }
            }
        }
        data.use { input ->
            val buf = ByteArray(1 shl 16)
            for (e in op.dstExtents) {
                dst.seek(e.startBlock * blockBytes)
                var remaining = e.numBlocks * blockBytes
                while (remaining > 0) {
                    val n = input.read(buf, 0, minOf(buf.size.toLong(), remaining).toInt())
                    // REPLACE data may be shorter than the extent; setLength() left the rest zeroed.
                    if (n < 0) break
                    dst.write(buf, 0, n)
                    remaining -= n
                }
            }
        }
    }

    private class ZeroStream(private var left: Long) : InputStream() {
        override fun read(): Int = if (left-- > 0) 0 else -1
        override fun read(b: ByteArray, off: Int, len: Int): Int {
            if (left <= 0) return -1
            val n = minOf(len.toLong(), left).toInt()
            b.fill(0, off, off + n)
            left -= n
            return n
        }
    }

    companion object {
        private val MAGIC = "CrAU".toByteArray()

        fun open(file: File): Payload = RandomAccessFile(file, "r").use { f ->
            val magic = ByteArray(4)
            f.readFully(magic)
            if (!magic.contentEquals(MAGIC)) throw FirmwareFormatException("not an update_engine payload")
            val version = f.readLong()
            if (version != 2L) throw FirmwareFormatException("unsupported payload version $version")
            val manifestSize = f.readLong()
            val metadataSigSize = f.readInt().toLong() and 0xFFFFFFFFL
            if (manifestSize <= 0 || manifestSize > 64L * 1024 * 1024) throw FirmwareFormatException("bad manifest size")
            val manifest = ByteArray(manifestSize.toInt())
            f.readFully(manifest)
            val dataOffset = 4 + 8 + 8 + 4 + manifestSize + metadataSigSize
            parseManifest(file, version, manifest, dataOffset)
        }

        /** Opens `payload.bin` directly from an OTA zip by extracting it next to [workDir]. */
        fun openFromOta(otaZip: File, workDir: File): Payload {
            java.util.zip.ZipFile(otaZip).use { zip ->
                val entry = zip.getEntry("payload.bin") ?: throw FirmwareFormatException("OTA has no payload.bin")
                val out = File(workDir, "payload.bin")
                if (!out.exists() || out.length() != entry.size) {
                    zip.getInputStream(entry).use { input -> out.outputStream().use { input.copyTo(it, 1 shl 16) } }
                }
                return open(out)
            }
        }

        private fun parseManifest(file: File, version: Long, bytes: ByteArray, dataOffset: Long): Payload {
            val r = ProtoReader(bytes)
            var blockSize = 4096
            var minor = 0
            val partitions = mutableListOf<Partition>()
            while (r.hasMore()) {
                val (field, wire) = r.readTag()
                when (field) {
                    3 -> blockSize = r.readVarint().toInt()
                    12 -> minor = r.readVarint().toInt()
                    13 -> partitions += parsePartition(r.readMessage())
                    else -> r.skip(wire)
                }
            }
            return Payload(file, version, blockSize, minor, partitions, dataOffset)
        }

        private fun parsePartition(r: ProtoReader): Partition {
            var name = ""
            var size = 0L
            var hash: ByteArray? = null
            val ops = mutableListOf<Operation>()
            while (r.hasMore()) {
                val (field, wire) = r.readTag()
                when (field) {
                    1 -> name = String(r.readBytes())
                    7 -> {
                        val info = r.readMessage()
                        while (info.hasMore()) {
                            val (f, w) = info.readTag()
                            when (f) {
                                1 -> size = info.readVarint()
                                2 -> hash = info.readBytes()
                                else -> info.skip(w)
                            }
                        }
                    }
                    8 -> ops += parseOperation(r.readMessage())
                    else -> r.skip(wire)
                }
            }
            return Partition(name, size, hash, ops)
        }

        private fun parseOperation(r: ProtoReader): Operation {
            var type = OpType.REPLACE
            var offset = 0L
            var length = 0L
            var hash: ByteArray? = null
            val dst = mutableListOf<Extent>()
            while (r.hasMore()) {
                val (field, wire) = r.readTag()
                when (field) {
                    1 -> type = OpType.of(r.readVarint().toInt())
                    2 -> offset = r.readVarint()
                    3 -> length = r.readVarint()
                    6 -> {
                        val e = r.readMessage()
                        var start = 0L
                        var num = 0L
                        while (e.hasMore()) {
                            val (f, w) = e.readTag()
                            when (f) {
                                1 -> start = e.readVarint()
                                2 -> num = e.readVarint()
                                else -> e.skip(w)
                            }
                        }
                        dst += Extent(start, num)
                    }
                    8 -> hash = r.readBytes()
                    else -> r.skip(wire)
                }
            }
            return Operation(type, offset, length, dst, hash)
        }

        internal fun sha256(input: InputStream): ByteArray {
            val md = MessageDigest.getInstance("SHA-256")
            val buf = ByteArray(1 shl 16)
            while (true) {
                val n = input.read(buf)
                if (n < 0) break
                md.update(buf, 0, n)
            }
            return md.digest()
        }
    }
}
