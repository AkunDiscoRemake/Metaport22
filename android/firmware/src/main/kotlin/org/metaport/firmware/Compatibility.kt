package org.metaport.firmware

/**
 * Works out, for each firmware binary, whether the phone can load it.
 *
 * A library resolves when every DT_NEEDED entry is either another firmware library that
 * resolves, a library the phone provides to apps, or a Metaport shim; and every undefined
 * symbol is exported by something in that closure. What is left over is exactly the list of
 * shims the port still needs — the work list of a real port.
 */
class CompatibilityAnalyzer(
    firmware: Collection<ElfInfo>,
    /** Libraries an app can link against on the phone (NDK public libraries), name -> exports. */
    private val phone: Map<String, Set<String>>,
    /** Libraries implemented by Metaport shims, name -> exports. */
    private val shims: Map<String, Set<String>> = emptyMap(),
) {
    enum class Status { LOADABLE, NEEDS_SHIMS, WRONG_ARCH }

    data class Report(
        val elf: ElfInfo,
        val status: Status,
        /** DT_NEEDED entries nobody provides (directly or transitively). */
        val missingLibraries: Set<String>,
        /** Undefined symbols nothing in the closure exports. */
        val missingSymbols: Set<String>,
    )

    private val byName: Map<String, ElfInfo> = firmware.associateBy { it.soname ?: it.path.substringAfterLast('/') }

    fun analyze(): List<Report> = byName.values.map(::analyze).sortedWith(compareBy({ it.status }, { it.elf.path }))

    fun analyze(elf: ElfInfo): Report {
        if (!elf.isArm64) return Report(elf, Status.WRONG_ARCH, emptySet(), emptySet())
        val missingLibs = sortedSetOf<String>()
        val available = mutableSetOf<String>()
        val seen = mutableSetOf<String>()
        val queue = ArrayDeque(elf.needed)
        while (queue.isNotEmpty()) {
            val lib = queue.removeFirst()
            if (!seen.add(lib)) continue
            when {
                lib in shims -> available += shims.getValue(lib)
                lib in byName -> {
                    val dep = byName.getValue(lib)
                    available += dep.exports
                    queue += dep.needed
                }
                lib in phone -> available += phone.getValue(lib)
                else -> missingLibs += lib
            }
        }
        val missingSyms = elf.imports.filterTo(sortedSetOf()) { it !in available && it !in elf.exports }
        val status = if (missingLibs.isEmpty() && missingSyms.isEmpty()) Status.LOADABLE else Status.NEEDS_SHIMS
        return Report(elf, status, missingLibs, missingSyms)
    }

    /** Libraries most often missing across the firmware: what to shim first. */
    fun shimPriorities(reports: List<Report> = analyze()): List<Pair<String, Int>> =
        reports.flatMap { it.missingLibraries }.groupingBy { it }.eachCount().toList().sortedByDescending { it.second }

    companion object {
        /** NDK "public" libraries every app on Android 10+ may load (bionic + stable NDK APIs). */
        val NDK_PUBLIC_LIBRARIES = setOf(
            "libc.so", "libm.so", "libdl.so", "liblog.so", "libz.so", "libandroid.so", "libEGL.so",
            "libGLESv1_CM.so", "libGLESv2.so", "libGLESv3.so", "libvulkan.so", "libjnigraphics.so",
            "libmediandk.so", "libcamera2ndk.so", "libnativewindow.so", "libOpenSLES.so", "libaaudio.so",
            "libbinder_ndk.so", "libsync.so", "libneuralnetworks.so", "libamidi.so", "libstdc++.so",
        )
    }
}
