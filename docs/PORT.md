# Como o port funciona e o que falta

## Por que não dá para "só rodar o firmware"

O Horizon OS é um Android arm64 com:

1. kernel e drivers do Qualcomm XR2 (display, GPU, DSP, câmeras de tracking);
2. serviços de sistema da Meta (runtime VR, compositor, tracking de cabeça/mãos/controles) que
   rodam com UID de sistema e falam com esses drivers e HALs;
3. apps (o shell Horizon e os jogos) que falam com os serviços via OpenXR/VrApi e Binder.

Num celular **sem root**, as camadas 1 e 2 não sobem: é outro SoC, outro kernel, e o app não
tem privilégio para registrar serviços de sistema. A camada 3 roda, porque apps de Quest são APKs
arm64 comuns, desde que alguém responda às chamadas OpenXR. É essa a função do
`libmetaport_openxr.so`.

O port então segue o modelo de emulação de alto nível (HLE):

```
 camada 3 (apps)          → roda direto; OpenXR atendido pelo runtime do Metaport
 camada 2 (serviços Meta) → carregados do firmware do usuário onde for possível,
                            com shims no lugar do que exige hardware/privilégio
 camada 1 (kernel/HAL)    → substituída: ARCore, IMU do celular, câmera, Joy-Con
```

## Fluxo com o firmware do usuário

1. **Importar:** o usuário escolhe o OTA dele no Vault (`android/vault`). É feito o hash e a
   catalogação; nada é baixado.
2. **Extrair:** `Payload` (`android/firmware`) lê o `payload.bin` e grava cada partição,
   verificando todos os hashes. Só OTAs completos; incrementais exigem a versão anterior.
3. **Listar arquivos:** hoje, montando a imagem só leitura no PC. *Próximo passo:* leitores de
   EROFS e ext4 em Kotlin para fazer isso no próprio celular.
4. **Analisar:** `CompatibilityAnalyzer` cruza os ELF do firmware com as bibliotecas públicas do
   celular do usuário e gera a lista exata de bibliotecas/símbolos que precisam de shim.
5. **Shimar e carregar:** os componentes `LOADABLE` (ou que viram loadable com shims) são
   carregados num namespace de linker isolado do processo do Metaport, a partir da cópia
   privada do usuário.

## Roteiro

| Etapa | Entrega |
|---|---|
| A | Leitor EROFS (incluindo clusters LZ4) e ext4 (extents) em `android/firmware` |
| B | App Android: serviço de tracking (ARCore, IMU, Joy-Con via `InputDevice`/acessibilidade), Vault na UI, launcher (abrir o app pelo Metaport também resolve a visibilidade de pacote do Android 11+) |
| C | Loader de componentes do firmware (`android_dlopen_ext` com namespace próprio) + infraestrutura de shim (stubs gerados a partir do relatório) |
| D | Bridge de hand tracking: alimentar o `HandLandmarkProvider` com o componente de mãos do firmware, se a análise mostrar que ele é shimável (depende de DSP/câmera) |
| E | Shell Horizon: idem para os apps de sistema do firmware |

As etapas D e E dependem do que o relatório da etapa 4 mostrar para o firmware real. Não dá para
garantir de antemão que os componentes da Meta são separáveis do hardware. Se não forem, o
`HandLandmarkProvider` aceita outra implementação sem mudar nada no resto do sistema.

## Compatibilidade de apps

- **OpenXR:** suportado pelo runtime. A descoberta do runtime usa o broker do Khronos; apps
  compilados com o loader da própria Meta podem não encontrá-lo (precisa ser testado app a app).
- **VrApi** (API antiga): não suportada. O Vault avisa.
- **Oculus Platform SDK** (verificação de compra): não é contornada. O Vault avisa.
