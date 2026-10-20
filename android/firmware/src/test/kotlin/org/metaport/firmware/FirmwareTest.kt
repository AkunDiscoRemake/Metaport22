package org.metaport.firmware

import org.apache.commons.compress.compressors.bzip2.BZip2CompressorOutputStream
import org.tukaani.xz.LZMA2Options
import org.tukaani.xz.XZOutputStream
import java.io.ByteArrayOutputStream
import java.io.DataOutputStream
import java.io.File
import java.nio.file.Files
import java.security.MessageDigest
import kotlin.test.Test
import kotlin.test.assertContentEquals
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/** Tiny protobuf writer, test-only, to build payload manifests. */
private class Proto {
    val out = ByteArrayOutputStream()
    fun varint(v: Long) {
        var x = v
        while (true) {
            if (x and 0x7FL.inv() == 0L) { out.write(x.toInt()); return }
            out.write(((x and 0x7F) or 0x80).toInt())
            x = x ushr 7
        }
    }
    fun int(field: Int, v: Long) = apply { varint((field shl 3).toLong()); varint(v) }
    fun bytes(field: Int, b: ByteArray) = apply { varint(((field shl 3) or 2).toLong()); varint(b.size.toLong()); out.write(b) }
    fun msg(field: Int, p: Proto) = bytes(field, p.out.toByteArray())
}

private fun sha(b: ByteArray) = MessageDigest.getInstance("SHA-256").digest(b)

class PayloadTest {
    private val block = 4096

    private fun buildPayload(dir: File, corruptOp: Boolean = false, incremental: Boolean = false): Pair<File, ByteArray> {
        // Expected 4-block image: [A-block][xz B-block][zero][bz2 C-block]
        val a = ByteArray(block) { 'A'.code.toByte() }
        val b = ByteArray(block) { (it % 251).toByte() }
        val c = ByteArray(block) { 'C'.code.toByte() }
        val image = a + b + ByteArray(block) + c

        val xz = ByteArrayOutputStream().also { XZOutputStream(it, LZMA2Options()).use { s -> s.write(b) } }.toByteArray()
        val bz = ByteArrayOutputStream().also { BZip2CompressorOutputStream(it).use { s -> s.write(c) } }.toByteArray()
        val blobs = a + xz + bz

        fun extent(start: Long, n: Long) = Proto().int(1, start).int(2, n)
        fun op(type: Int, off: Long, len: Long, dst: Long, data: ByteArray?) = Proto().int(1, type.toLong()).apply {
            if (data != null) { int(2, off); int(3, len); bytes(8, if (corruptOp) ByteArray(32) else sha(data)) }
            msg(6, extent(dst, 1))
        }
        val part = Proto()
            .bytes(1, "system".toByteArray())
            .msg(7, Proto().int(1, image.size.toLong()).bytes(2, sha(image)))
            .msg(8, op(0, 0, a.size.toLong(), 0, a))
            .msg(8, op(8, a.size.toLong(), xz.size.toLong(), 1, xz))
            .msg(8, op(6, 0, 0, 2, null))
            .msg(8, op(1, (a.size + xz.size).toLong(), bz.size.toLong(), 3, bz))
        if (incremental) part.msg(8, Proto().int(1, 4L).msg(6, extent(0, 1)))  // SOURCE_COPY
        val manifest = Proto().int(3, block.toLong()).msg(13, part).toByteArrayCompat()

        val f = File(dir, "payload.bin")
        DataOutputStream(f.outputStream()).use { o ->
            o.write("CrAU".toByteArray())
            o.writeLong(2)
            o.writeLong(manifest.size.toLong())
            o.writeInt(0)
            o.write(manifest)
            o.write(blobs)
        }
        return f to image
    }

    private fun Proto.toByteArrayCompat() = out.toByteArray()

    @Test
    fun extractsFullPayloadAndVerifiesHashes() {
        val dir = Files.createTempDirectory("payload").toFile()
        val (file, image) = buildPayload(dir)
        val p = Payload.open(file)
        assertTrue(p.isFull)
        assertEquals(listOf("system"), p.partitions.map { it.name })
        val out = File(dir, "system.img")
        p.extract("system", out)
        assertContentEquals(image, out.readBytes())
    }

    @Test
    fun rejectsCorruptedOperationData() {
        val dir = Files.createTempDirectory("payload").toFile()
        val (file, _) = buildPayload(dir, corruptOp = true)
        assertFailsWith<FirmwareFormatException> { Payload.open(file).extract("system", File(dir, "s.img")) }
    }

    @Test
    fun incrementalPayloadIsRefused() {
        val dir = Files.createTempDirectory("payload").toFile()
        val (file, _) = buildPayload(dir, incremental = true)
        val p = Payload.open(file)
        assertFalse(p.isFull)
        assertFailsWith<UnsupportedOperationException> { p.extract("system", File(dir, "s.img")) }
    }

    @Test
    fun rejectsNonPayload() {
        val f = Files.createTempFile("x", ".bin").toFile().apply { writeBytes(ByteArray(64)) }
        assertFailsWith<FirmwareFormatException> { Payload.open(f) }
    }
}

class ElfTest {
    private fun fixture(name: String): ElfInfo {
        val url = javaClass.getResource("/elf/$name") ?: error("missing fixture $name")
        return Elf.read(File(url.toURI()), "/system/lib64/$name")
    }

    @Test
    fun readsNeededImportsAndExports() {
        val foo = fixture("libfoo.so")
        assertTrue(foo.isArm64)
        assertEquals("libfoo.so", foo.soname)
        assertEquals(listOf("libbar.so", "liblog.so"), foo.needed)
        assertTrue("bar" in foo.imports && "__android_log_print" in foo.imports)
        assertTrue("foo" in foo.exports)
    }

    @Test
    fun compatibilityFindsWhatNeedsShims() {
        val libs = listOf(fixture("libfoo.so"), fixture("libbar.so"), fixture("libvrapp.so"))
        val phone = mapOf("liblog.so" to setOf("__android_log_print"))
        val reports = CompatibilityAnalyzer(libs, phone).analyze().associateBy { it.elf.soname }

        assertEquals(CompatibilityAnalyzer.Status.LOADABLE, reports.getValue("libfoo.so").status)
        assertEquals(CompatibilityAnalyzer.Status.LOADABLE, reports.getValue("libbar.so").status)
        val vr = reports.getValue("libvrapp.so")
        assertEquals(CompatibilityAnalyzer.Status.NEEDS_SHIMS, vr.status)
        assertEquals(setOf("libvrplatform.so"), vr.missingLibraries)
        assertEquals(setOf("vr_platform_init"), vr.missingSymbols)

        // Providing a shim for the missing library makes it loadable.
        val withShim = CompatibilityAnalyzer(libs, phone, shims = mapOf("libvrplatform.so" to setOf("vr_platform_init")))
        assertEquals(CompatibilityAnalyzer.Status.LOADABLE, withShim.analyze(vr.elf).status)
    }

    @Test
    fun missingPhoneLibraryIsReported() {
        val libs = listOf(fixture("libfoo.so"), fixture("libbar.so"))
        val r = CompatibilityAnalyzer(libs, phone = emptyMap()).analyze(libs[0])
        assertEquals(setOf("liblog.so"), r.missingLibraries)
        assertEquals(setOf("__android_log_print"), r.missingSymbols)
    }
}
