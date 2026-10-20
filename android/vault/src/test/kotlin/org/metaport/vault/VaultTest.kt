package org.metaport.vault

import java.io.ByteArrayOutputStream
import java.io.File
import java.nio.file.Files
import java.util.zip.ZipEntry
import java.util.zip.ZipOutputStream
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

class VaultTest {
    private fun zip(vararg entries: Pair<String, String>): ByteArray {
        val bos = ByteArrayOutputStream()
        ZipOutputStream(bos).use { z ->
            for ((name, content) in entries) {
                z.putNextEntry(ZipEntry(name))
                z.write(content.toByteArray())
                z.closeEntry()
            }
        }
        return bos.toByteArray()
    }

    private fun tempDir(): File = Files.createTempDirectory("vault").toFile()

    @Test
    fun questApkWithPlatformSdkIsFlagged() {
        val vault = Vault(tempDir())
        val apk = zip(
            "AndroidManifest.xml" to "x",
            "classes.dex" to "x",
            "lib/arm64-v8a/libopenxr_loader.so" to "x",
            "lib/arm64-v8a/libovrplatformloader.so" to "x",
        )
        val e = vault.import("game.apk", apk.inputStream())
        assertEquals(FileKind.QUEST_APK, e.kind)
        assertTrue(Warning.PLATFORM_ENTITLEMENT in e.warnings)
        assertTrue(vault.verify(e))
    }

    @Test
    fun vrApiOnlyAppIsMarkedIncompatible() {
        val i = Inspector.inspectApk(setOf("AndroidManifest.xml", "lib/arm64-v8a/libvrapi.so"))
        assertEquals(FileKind.QUEST_APK, i.kind)
        assertTrue(Warning.VRAPI_ONLY in i.warnings)
    }

    @Test
    fun otaIsCatalogedButNeverExecutable() {
        val vault = Vault(tempDir())
        val ota = zip(
            "payload.bin" to "data",
            "META-INF/com/android/metadata" to "ota-type=AB\npost-build=oculus/hollywood/hollywood:12/SQ3A/123:user/release-keys\n",
        )
        val e = vault.import("update.zip", ota.inputStream())
        assertEquals(FileKind.FIRMWARE_OTA, e.kind)
        assertEquals("oculus/hollywood/hollywood:12/SQ3A/123:user/release-keys", e.buildFingerprint)
        assertTrue(Warning.FIRMWARE_NOT_EXECUTABLE in e.warnings)
    }

    @Test
    fun obbNameGivesPackage() {
        val vault = Vault(tempDir())
        val e = vault.import("main.42.com.example.game.obb", "obbdata".byteInputStream())
        assertEquals(FileKind.OBB, e.kind)
        assertEquals("com.example.game", e.packageHint)
    }

    @Test
    fun deduplicatesAndPersists() {
        val dir = tempDir()
        val v1 = Vault(dir)
        val a = v1.import("a.bin", "same".byteInputStream())
        val b = v1.import("b.bin", "same".byteInputStream())
        assertEquals(a.sha256, b.sha256)
        assertEquals(1, v1.list().size)

        val v2 = Vault(dir)
        val loaded = v2.list().single()
        assertEquals(a, loaded)
        assertTrue(v2.remove(a.sha256))
        assertFalse(File(dir, a.storedName).exists())
        assertTrue(Vault(dir).list().isEmpty())
    }

    @Test
    fun tamperedFileFailsVerification() {
        val vault = Vault(tempDir())
        val e = vault.import("x.bin", "original".byteInputStream())
        vault.fileFor(e).writeText("changed")
        assertFalse(vault.verify(e))
    }

    @Test
    fun indexLineRoundTripsTabsSafely() {
        val e = VaultEntry("ab", Vault.sanitize("na\tme"), FileKind.UNKNOWN, 1, 2)
        val parsed = assertNotNull(Vault.parse(Vault.format(e)))
        assertEquals("na me", parsed.originalName)
    }
}
