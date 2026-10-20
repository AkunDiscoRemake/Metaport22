package org.metaport.vault

import java.io.File
import java.io.InputStream

/**
 * User content vault stored in the app's private directory.
 *
 * Import copies the user's file in, hashes it, classifies it and records it in `index.tsv`.
 * Files are deduplicated by SHA-256. Nothing is ever fetched from the network.
 */
class Vault(private val dir: File, private val clock: () -> Long = System::currentTimeMillis) {
    private val indexFile = File(dir, "index.tsv")
    private val entries = linkedMapOf<String, VaultEntry>()

    init {
        dir.mkdirs()
        load()
    }

    fun list(): List<VaultEntry> = entries.values.toList()

    fun fileFor(entry: VaultEntry): File = File(dir, entry.storedName)

    /** Imports a stream the user selected. Returns the (possibly pre-existing) entry. */
    fun import(originalName: String, input: InputStream): VaultEntry {
        val tmp = File.createTempFile("import", ".part", dir)
        try {
            tmp.outputStream().use { out -> input.copyTo(out, 1 shl 16) }
            val hash = tmp.inputStream().use(Inspector::sha256)
            entries[hash]?.let { return it }

            val inspection = Inspector.inspect(tmp, originalName)
            val entry = VaultEntry(
                sha256 = hash,
                originalName = sanitize(originalName),
                kind = inspection.kind,
                sizeBytes = tmp.length(),
                importedAtMillis = clock(),
                packageHint = inspection.packageHint,
                buildFingerprint = inspection.buildFingerprint,
                warnings = inspection.warnings,
            )
            if (!tmp.renameTo(fileFor(entry))) error("could not store ${entry.storedName}")
            entries[hash] = entry
            save()
            return entry
        } finally {
            tmp.delete()
        }
    }

    fun remove(sha256: String): Boolean {
        val e = entries.remove(sha256) ?: return false
        fileFor(e).delete()
        save()
        return true
    }

    /** Re-hashes a stored file; false means it was modified or corrupted on disk. */
    fun verify(entry: VaultEntry): Boolean {
        val f = fileFor(entry)
        return f.exists() && f.inputStream().use(Inspector::sha256) == entry.sha256
    }

    private fun load() {
        if (!indexFile.exists()) return
        indexFile.readLines().filter { it.isNotBlank() }.forEach { line ->
            parse(line)?.let { entries[it.sha256] = it }
        }
    }

    private fun save() {
        val tmp = File(dir, "index.tsv.tmp")
        tmp.writeText(entries.values.joinToString("\n", postfix = "\n") { format(it) })
        if (!tmp.renameTo(indexFile)) {
            indexFile.delete()
            tmp.renameTo(indexFile)
        }
    }

    companion object {
        private const val VERSION = "v1"

        internal fun sanitize(name: String) = name.replace(Regex("[\\t\\n\\r]"), " ").take(255)

        internal fun format(e: VaultEntry): String = listOf(
            VERSION, e.sha256, e.originalName, e.kind.name, e.sizeBytes.toString(), e.importedAtMillis.toString(),
            e.packageHint.orEmpty(), e.buildFingerprint.orEmpty(), e.warnings.joinToString(",") { it.name },
        ).joinToString("\t")

        internal fun parse(line: String): VaultEntry? {
            val f = line.split('\t')
            if (f.size < 9 || f[0] != VERSION) return null
            return try {
                VaultEntry(
                    sha256 = f[1],
                    originalName = f[2],
                    kind = FileKind.valueOf(f[3]),
                    sizeBytes = f[4].toLong(),
                    importedAtMillis = f[5].toLong(),
                    packageHint = f[6].ifEmpty { null },
                    buildFingerprint = f[7].ifEmpty { null },
                    warnings = f[8].split(',').filter { it.isNotEmpty() }.map { Warning.valueOf(it) }.toSet(),
                )
            } catch (e: IllegalArgumentException) {
                null
            }
        }
    }
}
