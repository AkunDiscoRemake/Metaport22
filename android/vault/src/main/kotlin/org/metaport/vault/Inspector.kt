package org.metaport.vault

import java.io.File
import java.io.InputStream
import java.security.MessageDigest
import java.util.zip.ZipFile

/** Result of looking inside a file, before it becomes a [VaultEntry]. */
data class Inspection(
    val kind: FileKind,
    val packageHint: String? = null,
    val buildFingerprint: String? = null,
    val warnings: Set<Warning> = emptySet(),
)

object Inspector {
    private val OBB_NAME = Regex("""^(main|patch)\.(\d+)\.([A-Za-z0-9_]+(?:\.[A-Za-z0-9_]+)+)\.obb$""")

    fun sha256(input: InputStream): String {
        val md = MessageDigest.getInstance("SHA-256")
        val buf = ByteArray(1 shl 16)
        while (true) {
            val n = input.read(buf)
            if (n < 0) break
            md.update(buf, 0, n)
        }
        return md.digest().joinToString("") { "%02x".format(it) }
    }

    /** Classifies a local copy of a user file. `originalName` is the name the user picked. */
    fun inspect(file: File, originalName: String): Inspection {
        val lower = originalName.lowercase()
        OBB_NAME.matchEntire(originalName)?.let { return Inspection(FileKind.OBB, packageHint = it.groupValues[3]) }
        if (lower.endsWith(".img")) {
            return Inspection(FileKind.FIRMWARE_IMAGE, warnings = setOf(Warning.FIRMWARE_NOT_EXECUTABLE))
        }
        val names = zipEntryNames(file) ?: return Inspection(FileKind.UNKNOWN)
        return when {
            "AndroidManifest.xml" in names -> inspectApk(names)
            "payload.bin" in names -> inspectOta(file, names)
            else -> Inspection(FileKind.UNKNOWN)
        }
    }

    internal fun inspectApk(names: Set<String>): Inspection {
        val libs = names.filter { it.startsWith("lib/") && it.endsWith(".so") }
        val arm64 = libs.filter { it.startsWith("lib/arm64-v8a/") }.map { it.substringAfterLast('/') }.toSet()
        val openxr = "libopenxr_loader.so" in arm64
        val vrapi = "libvrapi.so" in arm64
        val warnings = mutableSetOf<Warning>()
        if (arm64.any { it.startsWith("libovrplatformloader") || it.startsWith("libOVRPlatform") }) {
            warnings += Warning.PLATFORM_ENTITLEMENT
        }
        if (vrapi && !openxr) warnings += Warning.VRAPI_ONLY
        if (libs.isNotEmpty() && arm64.isEmpty()) warnings += Warning.NO_ARM64
        val kind = if (openxr || vrapi) FileKind.QUEST_APK else FileKind.ANDROID_APK
        return Inspection(kind, warnings = warnings)
    }

    internal fun inspectOta(file: File, names: Set<String>): Inspection {
        val metadata = if ("META-INF/com/android/metadata" in names) readZipText(file, "META-INF/com/android/metadata") else null
        val props = metadata?.let(::parseProperties).orEmpty()
        return Inspection(
            FileKind.FIRMWARE_OTA,
            buildFingerprint = props["post-build"],
            warnings = setOf(Warning.FIRMWARE_NOT_EXECUTABLE),
        )
    }

    internal fun parseProperties(text: String): Map<String, String> =
        text.lineSequence()
            .map { it.trim() }
            .filter { it.isNotEmpty() && !it.startsWith("#") && '=' in it }
            .associate { it.substringBefore('=') to it.substringAfter('=') }

    private fun zipEntryNames(file: File): Set<String>? = try {
        ZipFile(file).use { zip -> zip.entries().asSequence().map { it.name }.toSet() }
    } catch (e: Exception) {
        null  // not a zip
    }

    private fun readZipText(file: File, entry: String): String? = try {
        ZipFile(file).use { zip ->
            val e = zip.getEntry(entry) ?: return null
            if (e.size > 64 * 1024) return null
            zip.getInputStream(e).bufferedReader().readText()
        }
    } catch (e: Exception) {
        null
    }
}
