# Metaport

Camada de compatibilidade para rodar o software do **Meta Horizon OS** num celular Android dentro
de um óculos **Cardboard**, usando os **arquivos que o próprio usuário possui**: o firmware/OTA
do Quest dele e os apps que ele comprou.

> Projeto independente, sem afiliação com a Meta. "Meta", "Quest" e "Horizon OS" são marcas da
> Meta Platforms. O Metaport **não inclui, não baixa e não redistribui** nenhum arquivo da Meta.
> Veja [LEGAL.md](LEGAL.md).

## Estado atual (seja realista)

| Parte | Estado |
|---|---|
| Núcleo de tracking em C++: cabeça 6DoF (ARCore + giroscópio), fusão Joy-Con + mãos, hand joints do OpenXR | ✅ implementado, 31 testes |
| Runtime OpenXR (`libmetaport_openxr.so`): sessão, espaços, actions com perfil Oculus Touch, `XR_EXT_hand_tracking`, swapchains GLES, compositor Cardboard com distorção de lente e reprojeção rotacional | ✅ implementado, teste ponta a ponta; compila para arm64 com o NDK |
| Vault: importa, verifica (SHA-256) e cataloga os arquivos do usuário | ✅ 7 testes |
| Firmware: extrator de OTA `payload.bin` + analisador ELF + relatório de compatibilidade (o que carrega no celular, o que precisa de shim) | ✅ 7 testes + CLI |
| Leitor de sistema de arquivos (EROFS/ext4) no próprio celular | ⏳ por enquanto: montar a imagem no PC |
| Shims dos serviços do Horizon OS (runtime VR, tracking de mãos da Meta) | ⏳ é o trabalho principal do port, guiado pelo relatório acima |
| App Android (serviço de tracking, Joy-Con, UI) | ⏳ próximo passo |

**O que não dá para prometer:** o Horizon OS foi feito para o SoC XR2 do Quest, com o kernel, os
drivers, as câmeras de tracking e os privilégios de sistema dele. Num celular sem root esses
serviços não sobem como estão. Por isso o port funciona como um emulador HLE: cada dependência de
hardware vira um shim que traduz para o que o celular tem (câmera + ARCore, IMU, Joy-Con).
Detalhes e roteiro em [docs/PORT.md](docs/PORT.md).

## Arquitetura

```
 App de VR do Quest (APK do usuário)
        │  OpenXR
        ▼
 libmetaport_openxr.so  ──── lê ────►  memória compartilhada  ◄── escreve ── serviço Metaport
 (runtime, dentro do app)                (TrackingFrame)                    ├─ ARCore: cabeça 6DoF
        │                                                                   ├─ giroscópio do celular
        ▼                                                                   ├─ Joy-Con: IMU + botões
 compositor Cardboard                                                       └─ mãos: provider plugável
 (distorção + timewarp)                                                        (alvo: componente do firmware)
```

### Controles: Joy-Con + mãos = 6DoF estável

O Joy-Con dá orientação rápida, mas não posição; a câmera vê a mão que segura o Joy-Con e dá
posição absoluta, só que a ~30 Hz, atrasada e só quando a mão está no campo de visão. A fusão
(`core/src/controller_fusion.cpp`) faz o seguinte:

- **Orientação** só pela IMU do Joy-Con (filtro Mahony + aprendizado do bias do giroscópio em
  repouso). O yaw, que a IMU não observa, é alinhado aos poucos pela direção da palma.
- **Posição** prevista a 200 Hz pela aceleração sem gravidade, corrigida a cada frame da câmera
  **contra o estado no instante da imagem** (compensação de latência).
- **Sem teletransporte:** correções grandes viram um offset drenado a no máximo 2,5 m/s, então a
  reaquisição desliza em vez de pular.
- **Mão fora da câmera:** fica inercial por 150 ms e depois converge suavemente para um modelo de
  braço preso à cabeça. O controle nunca "voa" nem congela no ar.
- **Mão trocada:** se o detector confundir esquerda e direita, vence a geometria (a mão que está
  sobre o Joy-Con é a dele).
- **Sem Joy-Con:** a mão vira um controle Touch virtual (pinça = gatilho, punho = grip).
- **Joy-Con sem IMU** (depende do kernel do celular): pose vem da mão, botões vêm do Joy-Con.

## Compilar e testar

Núcleo + runtime (Linux/macOS):

```sh
cmake -S core -B build && cmake --build build -j
./build/metaport_tests && ./build/runtime/metaport_runtime_tests
```

Runtime para Android arm64:

```sh
cmake -S core -B build-android \
  -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake \
  -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-29
cmake --build build-android -j
```

Módulos JVM (vault e firmware):

```sh
cd android && ./gradlew :vault:test :firmware:test
```

## Usar com o SEU firmware

```sh
cd android && ./gradlew :firmware:installDist
F=firmware/build/install/metaport-firmware/bin/metaport-firmware

$F extract  ~/meu-quest-ota.zip  ~/fw/          # extrai e verifica as partições
sudo mount -o loop,ro ~/fw/system.img /mnt/fw    # monta só leitura
adb pull /system/lib64 ~/meu-celular-lib64       # bibliotecas públicas do SEU celular
$F analyze  /mnt/fw  ~/meu-celular-lib64  relatorio.tsv
```

O relatório lista cada binário do firmware como `LOADABLE`, `NEEDS_SHIMS` ou `WRONG_ARCH`, e
ordena as bibliotecas que faltam pelo número de dependentes, ou seja, a fila de trabalho do port.
