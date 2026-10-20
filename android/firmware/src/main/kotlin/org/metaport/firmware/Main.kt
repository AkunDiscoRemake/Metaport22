package org.metaport.firmware

import java.io.File
import kotlin.system.exitProcess

private const val USAGE = """
metaport-firmware — ferramentas para o SEU firmware do Horizon OS

  extract <ota.zip|payload.bin> <saida/> [particao...]
      Extrai imagens de partição de um OTA completo (verifica todos os hashes).

  analyze <firmware/> <libs-do-celular/> [relatorio.tsv]
      <firmware/>        árvore de arquivos do firmware (ex.: system.img montado só leitura)
      <libs-do-celular/> /system/lib64 do seu celular (adb pull /system/lib64)
      Mostra quais binários carregam no celular como estão e quais precisam de shims.
"""

fun main(args: Array<String>) {
    when (args.firstOrNull()) {
        "extract" -> extract(args.drop(1))
        "analyze" -> analyze(args.drop(1))
        else -> { System.err.println(USAGE); exitProcess(2) }
    }
}

private fun extract(args: List<String>) {
    if (args.size < 2) { System.err.println(USAGE); exitProcess(2) }
    val input = File(args[0])
    val out = File(args[1]).apply { mkdirs() }
    val payload = if (input.name.endsWith(".zip")) Payload.openFromOta(input, out) else Payload.open(input)
    println("payload v${payload.formatVersion}, bloco ${payload.blockSize}, ${if (payload.isFull) "completo" else "incremental"}")
    val wanted = args.drop(2).toSet()
    for (p in payload.partitions) {
        if (wanted.isNotEmpty() && p.name !in wanted) continue
        if (!p.isFull) { println("  ${p.name}: incremental, ignorada"); continue }
        print("  ${p.name} (${p.size / (1 shl 20)} MiB) ")
        payload.extract(p.name, File(out, "${p.name}.img")) { f -> if ((f * 100).toInt() % 10 == 0) print(".") }
        println(" ok")
    }
}

private fun loadElfs(root: File): List<ElfInfo> = root.walkTopDown()
    .filter { it.isFile && !java.nio.file.Files.isSymbolicLink(it.toPath()) && it.length() in 64..(512L shl 20) }
    .filter { runCatching { Elf.isElf(it) }.getOrDefault(false) }
    .mapNotNull { f -> runCatching { Elf.read(f, "/" + f.relativeTo(root).path) }.getOrNull() }
    .toList()

private fun analyze(args: List<String>) {
    if (args.size < 2) { System.err.println(USAGE); exitProcess(2) }
    val firmware = loadElfs(File(args[0]))
    val phoneLibs = loadElfs(File(args[1]))
        .filter { (it.soname ?: it.path.substringAfterLast('/')) in CompatibilityAnalyzer.NDK_PUBLIC_LIBRARIES }
        .associate { (it.soname ?: it.path.substringAfterLast('/')) to it.exports }
    val analyzer = CompatibilityAnalyzer(firmware, phoneLibs)
    val reports = analyzer.analyze()
    val counts = reports.groupingBy { it.status }.eachCount()
    println("${firmware.size} binários ELF no firmware, ${phoneLibs.size} bibliotecas públicas do celular")
    CompatibilityAnalyzer.Status.entries.forEach { println("  $it: ${counts[it] ?: 0}") }
    println("Bibliotecas a implementar como shim (por nº de dependentes):")
    analyzer.shimPriorities(reports).take(25).forEach { (lib, n) -> println("  %-40s %d".format(lib, n)) }
    args.getOrNull(2)?.let { path ->
        File(path).printWriter().use { w ->
            w.println("path\tstatus\tmissing_libraries\tmissing_symbols")
            reports.forEach { r ->
                w.println("${r.elf.path}\t${r.status}\t${r.missingLibraries.joinToString(",")}\t${r.missingSymbols.size}")
            }
        }
        println("Relatório: $path")
    }
}
