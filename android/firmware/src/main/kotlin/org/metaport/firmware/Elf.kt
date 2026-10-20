package org.metaport.firmware

import java.io.File
import java.io.RandomAccessFile
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Dynamic-linking view of an ELF shared object or executable: what it needs (DT_NEEDED, undefined
 * symbols) and what it provides (exported symbols). Used to work out which parts of the user's
 * firmware can be loaded on the phone as-is and which need a compatibility shim.
 */
data class ElfInfo(
    val path: String,
    val machine: Int,
    val is64: Boolean,
    val soname: String?,
    val needed: List<String>,
    val imports: Set<String>,
    val exports: Set<String>,
) {
    val isArm64 get() = machine == EM_AARCH64 && is64

    companion object {
        const val EM_AARCH64 = 183
        const val EM_ARM = 40
        const val EM_X86_64 = 62
    }
}

object Elf {
    private const val SHT_DYNSYM = 11
    private const val SHT_DYNAMIC = 6
    private const val DT_NEEDED = 1L
    private const val DT_SONAME = 14L

    fun isElf(file: File): Boolean = file.length() >= 64 && RandomAccessFile(file, "r").use { f ->
        val m = ByteArray(4)
        f.readFully(m)
        m[0] == 0x7F.toByte() && m[1] == 'E'.code.toByte() && m[2] == 'L'.code.toByte() && m[3] == 'F'.code.toByte()
    }

    fun read(file: File, displayPath: String = file.path): ElfInfo {
        val bytes = file.readBytes()
        return parse(bytes, displayPath)
    }

    fun parse(bytes: ByteArray, path: String): ElfInfo {
        if (bytes.size < 64 || bytes[0] != 0x7F.toByte() || bytes[1] != 'E'.code.toByte()) throw FirmwareFormatException("$path: not ELF")
        val is64 = bytes[4].toInt() == 2
        val bb = ByteBuffer.wrap(bytes).order(if (bytes[5].toInt() == 2) ByteOrder.BIG_ENDIAN else ByteOrder.LITTLE_ENDIAN)
        fun addr(off: Int): Long = if (is64) bb.getLong(off) else bb.getInt(off).toLong() and 0xFFFFFFFFL
        val machine = bb.getShort(18).toInt() and 0xFFFF
        val shoff = (if (is64) bb.getLong(0x28) else (bb.getInt(0x20).toLong() and 0xFFFFFFFFL)).toInt()
        val shentsize = bb.getShort(if (is64) 0x3A else 0x2E).toInt() and 0xFFFF
        val shnum = bb.getShort(if (is64) 0x3C else 0x30).toInt() and 0xFFFF

        data class Sec(val type: Int, val offset: Int, val size: Int, val link: Int, val entsize: Int)
        val secs = (0 until shnum).mapNotNull { i ->
            val o = shoff + i * shentsize
            if (o + shentsize > bytes.size) return@mapNotNull null
            if (is64) Sec(bb.getInt(o + 4), bb.getLong(o + 0x18).toInt(), bb.getLong(o + 0x20).toInt(), bb.getInt(o + 0x28), bb.getLong(o + 0x38).toInt())
            else Sec(bb.getInt(o + 4), bb.getInt(o + 0x10), bb.getInt(o + 0x14), bb.getInt(o + 0x18), bb.getInt(o + 0x24))
        }
        fun cstr(base: Int, off: Long): String {
            var e = base + off.toInt()
            val s = e
            while (e < bytes.size && bytes[e] != 0.toByte()) e++
            return String(bytes, s, e - s)
        }

        val imports = mutableSetOf<String>()
        val exports = mutableSetOf<String>()
        secs.filter { it.type == SHT_DYNSYM }.forEach { sym ->
            val str = secs.getOrNull(sym.link) ?: return@forEach
            val ent = if (sym.entsize > 0) sym.entsize else if (is64) 24 else 16
            for (i in 1 until sym.size / ent) {
                val o = sym.offset + i * ent
                val name = cstr(str.offset, bb.getInt(o).toLong() and 0xFFFFFFFFL)
                val info = bytes[o + if (is64) 4 else 12].toInt() and 0xFF
                val shndx = bb.getShort(o + if (is64) 6 else 14).toInt() and 0xFFFF
                val bind = info ushr 4
                if (name.isEmpty()) continue
                if (shndx == 0) { if (bind != 2) imports += name }  // weak undefined symbols are optional
                else if (bind == 1 || bind == 2) exports += name
            }
        }

        val needed = mutableListOf<String>()
        var soname: String? = null
        secs.filter { it.type == SHT_DYNAMIC }.forEach { dyn ->
            val str = secs.getOrNull(dyn.link) ?: return@forEach
            val ent = if (is64) 16 else 8
            for (i in 0 until dyn.size / ent) {
                val o = dyn.offset + i * ent
                val tag = addr(o)
                val v = addr(o + if (is64) 8 else 4)
                when (tag) {
                    0L -> break
                    DT_NEEDED -> needed += cstr(str.offset, v)
                    DT_SONAME -> soname = cstr(str.offset, v)
                }
            }
        }
        return ElfInfo(path, machine, is64, soname, needed, imports, exports)
    }
}
