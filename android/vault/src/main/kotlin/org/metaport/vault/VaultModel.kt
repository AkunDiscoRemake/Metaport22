package org.metaport.vault

/**
 * What a user-supplied file turned out to be.
 *
 * Metaport never ships, downloads or decrypts Meta software. Everything in the vault was
 * selected by the user from their own storage; the vault only identifies, hashes and catalogs it.
 */
enum class FileKind(val label: String) {
    /** An APK built for Quest (bundles an OpenXR loader / VrApi). */
    QUEST_APK("App de VR (APK)"),
    /** Any other Android APK. */
    ANDROID_APK("APK Android"),
    /** Expansion data for an app (`main.<version>.<package>.obb`). */
    OBB("Dados OBB"),
    /** An A/B OTA package (payload.bin + metadata), e.g. a Horizon OS update the user owns. */
    FIRMWARE_OTA("Firmware / OTA"),
    /** A raw partition image (*.img). */
    FIRMWARE_IMAGE("Imagem de partição"),
    UNKNOWN("Desconhecido"),
}

enum class Warning(val message: String) {
    /** The app links the Oculus Platform SDK: it will usually check a purchase entitlement with
     *  Meta's servers, which only works on a Quest. Metaport does not bypass this. */
    PLATFORM_ENTITLEMENT(
        "Usa o Oculus Platform SDK: o app verifica a compra nos servidores da Meta e normalmente " +
            "não abre fora de um Quest. O Metaport não contorna essa verificação."
    ),
    /** Uses the legacy VrApi instead of OpenXR; Metaport only implements OpenXR. */
    VRAPI_ONLY("Usa apenas VrApi (API antiga do Oculus), não OpenXR: não é compatível."),
    /** No arm64 native code: Quest apps are arm64; probably not a VR app at all. */
    NO_ARM64("Sem bibliotecas arm64-v8a."),
    /** Firmware cannot be booted on a phone; it is cataloged only. */
    FIRMWARE_NOT_EXECUTABLE(
        "Firmware do Horizon OS não roda em celular (outro SoC, drivers e boot assinado). " +
            "Ele fica catalogado e verificado (hash), mas não é executado."
    ),
}

data class VaultEntry(
    val sha256: String,
    val originalName: String,
    val kind: FileKind,
    val sizeBytes: Long,
    val importedAtMillis: Long,
    /** For APKs/OBBs: the Android package name when it can be determined from the file name. */
    val packageHint: String? = null,
    /** For OTAs: the `post-build` fingerprint from the package metadata. */
    val buildFingerprint: String? = null,
    val warnings: Set<Warning> = emptySet(),
) {
    /** Name of the copy inside the app's private vault directory. */
    val storedName: String get() = sha256 + extensionFor(kind)

    companion object {
        fun extensionFor(kind: FileKind) = when (kind) {
            FileKind.QUEST_APK, FileKind.ANDROID_APK -> ".apk"
            FileKind.OBB -> ".obb"
            FileKind.FIRMWARE_OTA -> ".zip"
            FileKind.FIRMWARE_IMAGE -> ".img"
            FileKind.UNKNOWN -> ".bin"
        }
    }
}
