package org.metaport.firmware

/**
 * Minimal protobuf wire-format reader: enough to decode update_engine's DeltaArchiveManifest
 * without generated code or a protobuf runtime dependency.
 */
class ProtoReader(private val buf: ByteArray, private var pos: Int = 0, private val end: Int = buf.size) {
    fun hasMore() = pos < end

    fun readVarint(): Long {
        var result = 0L
        var shift = 0
        while (true) {
            if (pos >= end) throw FirmwareFormatException("truncated varint")
            val b = buf[pos++].toInt() and 0xFF
            result = result or ((b and 0x7F).toLong() shl shift)
            if (b and 0x80 == 0) return result
            shift += 7
            if (shift > 63) throw FirmwareFormatException("varint too long")
        }
    }

    /** Returns (fieldNumber, wireType). */
    fun readTag(): Pair<Int, Int> {
        val t = readVarint()
        return (t ushr 3).toInt() to (t and 7).toInt()
    }

    fun readBytes(): ByteArray {
        val len = readVarint()
        if (len < 0 || len > end - pos) throw FirmwareFormatException("bad length $len")
        val out = buf.copyOfRange(pos, pos + len.toInt())
        pos += len.toInt()
        return out
    }

    fun readMessage(): ProtoReader {
        val len = readVarint().toInt()
        if (len < 0 || len > end - pos) throw FirmwareFormatException("bad message length $len")
        val r = ProtoReader(buf, pos, pos + len)
        pos += len
        return r
    }

    fun skip(wireType: Int) {
        when (wireType) {
            0 -> readVarint()
            1 -> pos += 8
            2 -> { val len = readVarint().toInt(); pos += len }
            5 -> pos += 4
            else -> throw FirmwareFormatException("unsupported wire type $wireType")
        }
        if (pos > end) throw FirmwareFormatException("truncated field")
    }
}

class FirmwareFormatException(message: String) : Exception(message)
