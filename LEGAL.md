# Uso legal

O Metaport foi desenhado para ser usado **só com arquivos que o usuário já possui legitimamente**.

## O que o Metaport faz

- Lê arquivos que o **usuário escolhe** no próprio armazenamento: o OTA/firmware do Quest dele e
  APKs/OBBs de apps que ele comprou.
- Verifica integridade (SHA-256) e cataloga esses arquivos numa pasta privada do app.
- Extrai partições de OTAs **completos**, que guardam os dados sem criptografia (raw/xz/bz2), e
  confere cada operação contra os hashes do próprio manifesto.
- Analisa binários (dependências, símbolos) para saber o que precisa de shim. Esse tipo de
  análise para interoperabilidade é protegido em várias jurisdições (por exemplo, art. 6 da
  Diretiva 2009/24/CE na UE e 17 U.S.C. §1201(f) nos EUA), mas a lei varia de país para país.
- Implementa APIs abertas (OpenXR, da Khronos) com código próprio.

## O que o Metaport NÃO faz e não vai fazer

- **Não inclui, não baixa e não redistribui** firmware, bibliotecas, apps ou assets da Meta.
  Os arquivos do usuário nunca saem do aparelho dele. O `.gitignore` bloqueia imagens de firmware.
- **Não contorna DRM nem verificação de compra.** Apps que verificam a compra nos servidores da
  Meta (Oculus Platform SDK) normalmente não abrem fora de um Quest. O Metaport avisa e para aí.
- **Não quebra criptografia nem assinatura** de boot, de partição ou de app.
- **Não usa** chaves, certificados nem credenciais da Meta.

## Responsabilidade do usuário

Os termos de licença do Horizon OS e dos apps podem restringir o uso fora do hardware da Meta.
Uma violação de licença é uma questão contratual entre você e a Meta, independente da legalidade
da ferramenta. Leia os termos e decida. Em caso de dúvida, consulte um advogado da sua jurisdição.
