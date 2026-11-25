# FRC 9460 — Pivô do elevador

Projeto Java com WPILib/GradleRIO 2026.2.1 e Phoenix 6 26.3.0.
O pivô usa TalonFX principal CAN 38 e seguidor CAN 39.
Não há um subsistema separado de elevação linear neste projeto.
O swerve (4 MAXSwerve de 3", 8 Spark MAX e um Pigeon 2) está na seção
**Swerve (MAXSwerve 3")**, mais abaixo.

## Organização dos subsistemas

Cada mecanismo reúne sua implementação, interfaces IO e constantes na própria pasta:

| Pasta em `src/main/java/frc/robot/subsystems` | Constantes do mecanismo |
| --- | --- |
| `elevatorpivot/` | `ElevatorPivotConstants.java` |
| `manipulador/` | `ManipuladorConstants.java` |
| `swerve/` | `SwerveConstants.java` |

O arquivo `src/main/java/frc/robot/Constants.java` contém as configurações gerais
do controle Xbox (`OperatorConstants`). Os testes do pivô ficam em
`src/test/java/frc/robot/subsystems/elevatorpivot/`.

## Operação

Movimento em **Teleop ou Test habilitado**, depois do zero manual.
O modo autônomo não movimenta o pivô. A restrição anterior a Test impedia
o funcionamento dos comandos durante o uso em Teleop.

> ⚠️ 19/11/2026: o pivô está desligado (`kEnabled = false`) e **Y, A e START são do
> swerve** (ver "Pilotagem no Xbox"). Ao ligar o pivô, trocar os botões de um dos dois.

| Controle Xbox, porta 0 | Ação |
| --- | --- |
| START, robô desabilitado | Zerar com o pêndulo fechado/de pé na marca física de referência |
| RB/R1 pressionado | Abaixar/abrir, em direção ao limite negativo |
| RT/R2 pressionado acima de 50% | Levantar/fechar, em direção a zero |
| Soltar RB/RT ou apertar ambos juntos | Manter o alvo capturado |
| A (um toque) | Ir para a posição A: fechado, 0° |
| B (um toque) | Ir para a posição B: meio, 50° de abertura |
| X (um toque) | Ir para a posição X: fim, 101° de abertura |
| Y pressionado | Aplicar saída neutra nos dois motores, com prioridade sobre todos |
| Soltar Y | Retomar RB/RT se pressionado, ou manter a posição atual |

A parada (neutro) mudou do B para o Y, porque o B agora é a posição do meio.
Depois de um toque em A, B ou X, o pivô vai até a posição e continua segurando
nela até RB/RT, Y ou outra posição assumir. Com Y pressionado, A/B/X não fazem nada.

START não procura fim de curso nem confirma a posição física.
Saída neutra com Brake não equivale a sustentação ativa.
Desabilitar o robô também remove a sustentação ativa.

Para operar após enviar o código: colocar o Xbox na porta 0 da Driver Station,
posicionar o pêndulo fechado, desabilitar e tocar START. Conferir
`ElevatorPivot/Zerado`, depois habilitar Teleop e usar RB/RT.
Se não movimentar, conferir `ElevatorPivot/Bloqueio` e `ElevatorPivot/Status`.

## Onde ajustar a velocidade

Editar estas linhas em `src/main/java/frc/robot/subsystems/elevatorpivot/ElevatorPivotConstants.java`,
na classe `ElevatorPivotConstants`, região **🚀 Ajuste de velocidade — abrir e fechar**,
e enviar o código novamente:

```java
public static final double kOpenSpeedDegPerSec = 100.0;  // R1/RB: abrir
public static final double kCloseSpeedDegPerSec = 100.0; // R2/RT: fechar
```

Os valores são graus do pivô por segundo e devem ser positivos.
Cada sentido pode ser ajustado independentemente. RT atua como botão acima de 50% do curso;
o quanto você aperta não altera a velocidade programada.

O pivô usa 2 Kraken X44. Com a redução de 405:1, a velocidade máxima
sem carga é cerca de 115°/s (7758 RPM, comutação trapezoidal) ou 109°/s
com FOC (7368 RPM). 100°/s é ~87% do máximo: sobra saída para o controle
manter a mesma velocidade ao fechar contra a gravidade, com ou sem FOC.
O valor anterior, 112,32°/s, pedia ~98% e deixava o motor quase sempre no
limite; numa simulação simples era só ~0,06 s mais rápido por movimento.
Valores acima de ~115 não aceleram mais; o motor anda no máximo que consegue.
Para ver a velocidade real, ler `VelocidadeRealGrausPorSegundo` no painel
durante um movimento.

### kV: o que faz o pivô chegar na velocidade

O controle de velocidade (Slot 1) usa kP e kV. Sem kV, o kP sozinho só
aumenta a saída quando já está atrasado, e o pivô fica bem abaixo do alvo.
O kV é calculado em `ElevatorPivotConstants.java` a partir da rotação livre do motor:

```java
public static final double kMotorFreeSpeedRpm = 7758.0; // Kraken X44, trapezoidal
public static final double kVelocityKv = kGearRatio / (kMotorFreeSpeedRpm / 60.0); // ~3,13
```

7758 RPM vem da tabela de desempenho do Kraken X44 na documentação da WCP.
Se o robô usar FOC (licença Phoenix Pro), a rotação livre cai para 7368 RPM.
Se os motores forem trocados, ajustar `kMotorFreeSpeedRpm` para o novo motor.
Com o kV, um alvo acima da velocidade máxima pede 100% de saída no meio do curso.
Numa simulação simples, sem gravidade, o movimento completo caiu
de ~1,6 s para ~1,1 s. Confirmar no robô.

No vídeo de 05/11/2026, com teto de 100%, o pivô abriu em ~1,15 s e fechou
em ~1,3 s (cerca de 85–90°/s e 75–80°/s em média), chegando às pontas sem
desacelerar. Por isso foi criada a zona de desaceleração abaixo.

## Desaceleração perto dos limites

Na região **🐢 Desaceleração perto dos limites** de `ElevatorPivotConstants.java`:

```java
public static final double kSlowZoneDeg = 15.0;               // tamanho da zona
public static final double kSlowZoneMinSpeedDegPerSec = 15.0; // velocidade na chegada
```

Nos últimos 15° antes do limite do sentido do movimento, o alvo cai em linha
reta da velocidade cheia até 15°/s. Ao sair do limite, no sentido oposto,
a velocidade é cheia. `kSlowZoneDeg = 0` desliga a rampa.
Enquanto desacelera, `ElevatorPivot/Status` mostra "desacelerando perto do limite".

Os valores são ponto de partida: conferir no robô, em Test, se a chegada
fica suave. Se ainda bater, aumentar a zona; se ficar lento demais no final,
diminuir a zona ou subir a velocidade de chegada.

O alvo é convertido para voltas do pivô por segundo dividindo por 360.
A redução já está configurada no TalonFX e não é aplicada novamente ao alvo.
O limite físico do motor, o teto de saída e a carga podem impedir atingir o alvo.
Compare `VelocidadeAlvoGrausPorSegundo`, `VelocidadeRealGrausPorSegundo`
e `SaidaPorcento` no painel. As constantes de velocidade definem o alvo;
os valores de ajuda e os ganhos do PID têm funções diferentes.

O teto fica em `kMaxDutyCycle = 1`, na região **🔋 Teto de saída**.
100% é a saída máxima permitida; a velocidade desejada é outra constante.
Com 100%, o limite de corrente de 18 A é o que segura o tranco na partida.
A malha fechada ajusta a saída usando o encoder para buscar o alvo dentro
desse teto. Não existe conversão fixa de porcentagem para graus por segundo.
O mesmo teto vale para manter a posição ao soltar os comandos.

As ajudas medidas continuam em `kUpFeedForwardDutyCycle = 0.08` e
`kDownFeedForwardDutyCycle = 0.03`. A validação rejeita ajudas acima do teto.
Depois de alterar os valores, enviar o código novamente e refazer o zero
com o pêndulo fechado, robô desabilitado e START.

## Posições fixas — A, B e X

Na região **📍 Posições fixas** de `ElevatorPivotConstants.java`, em graus de abertura
a partir do zero (fechado/de pé):

```java
public static final double kPositionADeg = 0.0;   // A: fechado (zero)
public static final double kPositionBDeg = 50.0;  // B: meio
public static final double kPositionXDeg = 101.0; // X: fim (aberto)
public static final double kPositionToleranceDeg = 1.0;
```

O pivô usa a mesma velocidade de RB/RT e desacelera nos últimos 15° antes
do alvo (a mesma rampa dos limites). A menos de 1° do alvo, passa a segurar
a posição exata. Valores fora do curso (0 a ~101,3°) param no limite.

Para ajustar, mover o pivô com RB/RT até a posição desejada e ler
`ElevatorPivot/AberturaGraus` no painel. Colocar esse número na constante.

### Folga da montagem

No registro do Tuner de 05/11/2026 (`tuner-plot.csv`), mexendo o pêndulo com a
mão, o rotor do CAN 38 variou só 0,0034 volta (0,418 a 0,421), o que equivale
a ~0,003° no pêndulo. O motor não percebe esse movimento: a folga está na
transmissão, entre o motor e o pêndulo. O código não consegue ver nem corrigir
essa folga pelo encoder do motor.

Na prática, as posições repetem no motor, mas o pêndulo pode ficar em qualquer
ponto dentro da folga. No meio e no fim, a gravidade empurra sempre para o mesmo
lado e a posição tende a repetir. No zero (de pé), a gravidade quase não age e o
pêndulo pode balançar dentro da folga. Para eliminar, reduzir a folga na montagem
ou medir o ângulo direto no eixo do pivô com um encoder absoluto.

## Regiões para manutenção

Os arquivos Java em `src/main/java` usam `// #region` e `// #endregion`
com ícones e nomes em português. No VS Code do WPILib, use a seta à esquerda
da linha da região para recolher ou expandir o trecho.

| Onde procurar | Conteúdo |
| --- | --- |
| `ElevatorPivotConstants.java` | 🚀 Velocidades, 🐢 desaceleração, 📍 posições A/B/X, 📏 limites, 🎯 ganhos, 💪 ajuda e 🔋 teto de saída |
| `RobotContainer.java` | 🎮 Botões do Xbox e comando padrão |
| `commands/ElevatorPivotCommands.java` | ↕️ Movimento, 📍 posições fixas, 🎯 sustentação e 🛑 parada |
| `subsystems/elevatorpivot/ElevatorPivot.java` | 📍 Zero, 🛡️ condições de movimento e 📊 painel |
| `subsystems/elevatorpivot/ElevatorPivotIO.java` e `ElevatorPivotIOTalonFX.java` | 🔌 Motores, 📡 leituras e configuração |
| `subsystems/swerve/SwerveConstants.java` | 🛠️ IDs e inversões, 📏 medidas e reduções, 🛑 limites, 🎯 ganhos, 🎮 pilotagem e 🧪 simulação |
| `subsystems/swerve/Swerve.java` e `SwerveModulo.java` | 🎯 Movimento, 🧭 pose e frente, 🛑 segurança e 📊 painel |
| `subsystems/swerve/SwerveIOSparkMax.java`, `SwerveGyroIOPigeon2.java` e `*IOSim.java` | ⚙️ Configuração dos Spark/Pigeon e 🧪 física da simulação |
| `commands/swerve/SwerveDirigirCommand.java` | 🎮 Analógicos, zona morta, modo lento e limite de aceleração |
| `Robot.java` e `Main.java` | 🚀 Inicialização e modos do robô |

## Valores medidos em 05/11/2026

A equipe confirmou zero físico com o pêndulo fechado/de pé e que os registros
representam o curso testado até as posições seguras de fechado e aberto.
Os valores estão em `ElevatorPivotConstants`.

| Parâmetro | Valor | Unidade / interpretação |
| --- | --- | --- |
| Redução | 9 × 9 × 5 = 405:1 | Duas etapas de 9:1 e uma de 5:1 |
| Limite fechado | 0 | Voltas do rotor relativas ao zero físico |
| Limite aberto | −114 | Voltas do rotor relativas ao zero físico |
| Ajuda para levantar/fechar | +0,08 | Fração de saída no principal: 8% |
| Ajuda para abaixar/abrir | −0,03 | Fração de saída no principal: −3% |
| Teto de saída configurado | ±1,00 | PID, ajuda e sustentação limitados a 100% em cada motor |

O teto de 100% foi escolhido pela equipe para o pivô andar mais rápido; não é
uma nova medição de inércia. O teto anterior era 30%.

As imagens do Tuner mostram DutyCycleOut: 0,03 significa 3% e 0,08 significa 8%.
São percentuais medidos para vencer a inércia nas condições testadas.
Não são posições, volts, nem uma medição da saída necessária para sustentação.

### Origem do limite

Fonte usada para os limites: `C:\Users\Admin\Desktop\Nova pasta\tuner-plot.csv`,
coluna `Pivot (Device ID 38) - RotorPosition`.

| Ponto no registro do principal | Leitura do rotor |
| --- | --- |
| Inicial, tomado como referência fechada conforme confirmação da equipe | −8,292969 |
| Extremo aberto observado | −122,419922 |
| Deslocamento aberto relativo ao fechado | −114,126953 |
| Limite adotado, arredondado para dentro do trecho registrado | −114,0 |

O offset absoluto antigo não é aplicado ao robô atual. Posicionar fisicamente
fechado e usar START, com o robô desabilitado, define novamente o zero.

O outro arquivo, `C:\Users\Admin\Desktop\tuner-plot.csv`, registra o seguidor
CAN 39 em um trecho diferente. Os extremos de motores/sessões diferentes não
foram misturados para definir o limite do principal.

**As medições já foram feitas com as reduções instaladas.** Como a coluna é
RotorPosition, ela continua representando voltas do motor. O código configura
SensorToMechanismRatio = 405 e converte os limites do rotor para o mecanismo uma única vez.
Não multiplicar as leituras do CSV novamente pela redução.

- −114 voltas do rotor / 405 = −0,281481 volta do pivô, aproximadamente −101,33°.
- O alvo inicial de 7,2°/s / 360 = 0,02 volta/s do pivô equivale a 8,1 voltas/s do rotor.
- Para outras medições: voltas do rotor = graus relativos ao fechado × 405 / 360.

O arredondamento de −114,126953 para −114 deixa somente cerca de 0,113° dentro
do extremo registrado. Isso não determina a folga mecânica necessária:
conferir no mecanismo a margem para batentes, cabos e colisões.

## Como os percentuais entram no controle

Subida e descida usam `VelocityDutyCycle`: o controle busca a velocidade alvo
e recebe ajuda de +8% ao levantar ou −3% ao abrir. A ajuda permanece durante
o movimento; não foi criado um pulso temporizado para a partida.
A saída efetiva varia com o controle e é limitada pelo teto, incluindo a ajuda.
Isso é malha fechada: o TalonFX compara o alvo com a velocidade do encoder
e calcula a saída pelo kV e pelo kP do Slot 1, conforme a
[API VelocityDutyCycle](https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/controls/VelocityDutyCycle.html).

Sustentação usa `PositionDutyCycle`, com alvo fixo e o mesmo teto de saída.
Os ganhos proporcionais foram convertidos de 60 para 5, passando da unidade
de volts para fração de saída com referência nominal de 12 V.
Continuam provisórios: os testes de inércia não comprovam sustentação,
velocidade atingível, estabilidade do controle ou comportamento sob carga.
Não há compensação de gravidade calibrada.

CAN 39 usa `Follower` com `MotorAlignmentValue.Opposed`, reproduzindo a
relação de sinais opostos dos testes no Tuner. A inversão configurada no
principal é preservada. A direção durante o seguimento é definida em relação
a esse principal, conforme a
[documentação da CTRE](https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/controls/Follower.html).
A posição dos dois motores, a velocidade e o DutyCycle do principal atualizam a 50 Hz.
O limite por posição fica no principal; o seguidor acompanha a saída.

## Proteções e painel

- Movimento depende de configuração válida, zero manual e Teleop ou Test habilitado.
- Leituras dos dois motores precisam ter status válido, valor finito e idade máxima de 100 ms.
- Reset de qualquer motor, perda de leitura durante controle ativo ou falha em
  uma solicitação de movimento/sustentação aplica neutro e exige novo zero.
- Falha do seguidor impede enviar nova solicitação de movimento ao principal.
- Todos os comandos reservam o subsistema. RB+RT mantém posição e Y tem prioridade.
- START mantido durante troca de modo não cria um novo evento de zero.
- Sem encoder absoluto ou sensor de referência, reposicionar fisicamente antes
  de refazer o zero depois de reset.

O painel mostra Calibracao, Configurado, Zerado, Pronto, LeiturasOK, Estado,
Status, Bloqueio, Posicao, PosicaoSeguidor, Alvo, correntes e tensão.
`VelocidadeAlvoGrausPorSegundo` e `VelocidadeRealGrausPorSegundo` permitem
comparar o alvo do controle com a leitura do encoder.
`SaidaPorcento` e `SaidaSeguidorPorcento` mostram a saída efetiva em porcentagem.
As posições do painel são voltas do pivô após o zero, não voltas do rotor.
Não há detecção calibrada de desalinhamento mecânico entre os dois motores.

## Manipulador (motores reais e simulação)

O subsistema `subsystems/manipulador` usa a física do projeto
`FRC9460_Calibracao` para simular a garra com mola, as rodas e o giro.
O CAD **Montagem geral rev1** inclui corpo, dois braços, dois conjuntos de rodas
e a esfera. A esfera de 15 cm aparece entre as rodas quando a garra a segura.

Motores do manipulador (informados pela equipe em 11/11/2026), em
`ManipuladorConstants.java`:

| Parte | Motor | CAN |
| --- | --- | --- |
| Garra (fecha; a mola abre) | Kraken X60 (TalonFX), em cima do manipulador | 42 |
| Rodas flexíveis (puxam o objeto) | Kraken X60 (TalonFX), em cima, ao lado do da garra | 43 |
| Giro da garra | NEO Vortex (Spark Flex) | 41 |

Os dois X60 estão montados do mesmo jeito (em cima, lado a lado), então giram
no **mesmo sentido** e usam a mesma inversão (`TOP_MOTORS_INVERTED`). O CAD
desenha um deles embaixo do corpo; o modelo 3D mostra os dois em cima, como no
mecanismo montado. No roboRIO o `Manipulador` usa os motores reais
(`ManipuladorGarraIOTalonFX`, `ManipuladorRodasIOTalonFX` e
`ManipuladorGiroIOSparkFlex`); no computador, os modelos simulados.
Sentidos, reduções e ganhos continuam marcados com 🚧 para conferir no mecanismo.

### Motores reais — limites de segurança provisórios

Os limites abaixo foram tirados da simulação em 12/11/2026, com folga, para os
primeiros testes nos motores reais. **Trocar pelos valores testados** em
`ManipuladorConstants.java`.

| Limite | Valor provisório | De onde veio |
| --- | --- | --- |
| Corrente da garra (estator e supply) | 25 A (antes 40) | Fecha vazia com pico de 15,5 A na simulação |
| Aperto parado da garra (`CLAW_HOLD_VOLTS`) | 0,6 V ≈ 18 A (antes 1 V ≈ 30 A) | Fica abaixo dos 25 A; a simulação segura o objeto com 18,3 A |
| Soft limit de fechar da garra | 34° (meta 33°) | 1° além da meta e antes das rodas se encostarem (~34,6°) |
| Corrente das rodas | estator 40 A, supply 30 A (antes 60/40) | A simulação pede 2,5 A sem carga; ela não simula o objeto |
| Corrente do giro | 80 A (padrão do Spark Flex; antes 60) | A equipe pediu mais força; com o motor quase parado, a força é a corrente (60 A já cortava acima de ~30% de duty). A carga medida pede ≈ 32 A parado no fim |
| Tempo máximo para fechar a garra | 1,0 s | A simulação fecha em 0,28 a 0,36 s |
| Tempo máximo para o giro chegar | 3,0 s | Giro por posição (MAXMotion): 0° → 95,6° em ~1,25 s na simulação, com a carga medida |

- **Garra demorou demais:** a garra é solta (a mola abre) e
  `Manipulador/Garra/Falha` mostra `TEMPO_ESGOTADO_AO_FECHAR`. Apertar → de novo
  tenta outra vez.
- **Giro demorou demais:** o giro para e `Manipulador/Giro/Estado` mostra
  `TEMPO_ESGOTADO`. Ele só tenta de novo depois de soltar o botão.
- A garra e o giro já são configurados ao ligar o robô: Coast na garra, Brake no
  giro, limites de corrente e soft limits valem antes do zero. O Spark Flex grava
  a configuração, para mantê-la se reiniciar no meio do jogo.
- O zero do giro só é perdido por um alerta **novo** do Spark (brownout, sensor ou
  reinício) depois do zero, ou por um salto impossível do encoder.

### Giro com motor direto (1:1)

O NEO Vortex gira o manipulador **direto, sem redução** (confirmado pela equipe em
12/11/2026; o valor antigo de 25:1 mandaria o motor dar 6,25 voltas para 90°).
Medições no REV Hardware Client, Spark 41:

| Medição | Valor | Uso no código |
| --- | --- | --- |
| Fim do giro (girando com a mão) | encoder de 0,02846 no 0 a 0,280134 no fim: 0,2517 volta = 90,6° | `ROTATION_MEASURED_END_MOTOR_ROTATIONS`; o fim usado é 5° além (`ROTATION_END_EXTRA_DEGREES`) = **95,6°** |
| Saída que tira da inércia | 0,01 (1%) | `ROTATION_KS` = 0,12 V |
| Saída para conseguir girar | 0,15 (15%) | Carga de ~0,54 N·m (~32 A parado) |

- O curso é a **diferença** entre as duas leituras: o código zera o encoder na
  posição 0 com BACK, desabilitado e na referência física (veja "Zero manual" abaixo).
- Fim do giro: 90,6° medido + 5° pedidos pela equipe = **95,6°**.
- Soft limits: 5° além do 0 e além do fim (−5° e 100,6°).

**Giro em malha fechada nos botões (cima/baixo segurados):**

- Cima pede a posição de 95,6°; baixo pede 0°. O Spark Flex usa o encoder e
  MAXMotion para acelerar, seguir o perfil e desacelerar até a posição.
- Ao soltar, o código calcula um alvo de parada pela velocidade atual e pela
  desaceleração configurada, limitado ao curso operacional. Mantém esse alvo
  com controle de posição; chamadas repetidas de BACK não deslocam o alvo.
- O teste completo usa o mesmo controle. Não há mais saída fixa de giro manual
  nem alternância entre aceleração e frenagem por duty cycle no roboRIO.
- Os alvos ficam entre 0° e 95,6°. Os soft limits de -5° e 100,6° são proteção
  adicional, não posições de operação.
- Chegada exige erro de até 2° e velocidade de até 5°/s. Se não chegar em 3 s,
  aplica neutro, registra `TEMPO_ESGOTADO` e exige um novo toque no botão.
- Perda de zero, desconexão ou velocidade inválida interrompe o controle.
  Recuperar uma leitura não retoma o movimento automaticamente.

| Ajuste em `ManipuladorConstants.java` | Atual | Unidade |
| --- | --- | --- |
| `ROTATION_CRUISE_ROTATIONS_PER_SEC` | 0,35 | Voltas do rotor/s: 126°/s com 1:1 |
| `ROTATION_ACCELERATION_ROTATIONS_PER_SEC2` | 0,60 | Voltas do rotor/s²: 216°/s² com 1:1 |
| `ROTATION_MAX_OUTPUT` | 0,60 | Teto de saída do controlador: 60% |
| `ROTATION_CURRENT_LIMIT_AMPS` | 80 | Limite de corrente do Spark, preservado |

O giro pelos botões agora usa o perfil de 126°/s já existente no controle de
posição, substituindo o antigo limitador manual de 200°/s. Os ganhos existentes
foram preservados; a simulação não calibra a carga e a inércia do mecanismo real.
Aumentar corrente ou saída não substitui o ajuste de velocidade e aceleração.

Telemetria: `Manipulador/Giro/MetaGraus`, `AnguloGraus`, `ErroGraus`,
`VelocidadeGrausPorSegundo`, `Tensao`, `Estado` e `Falha`.
`MAXMOTION`, `FREANDO_E_SEGURANDO` e `SEGURANDO` identificam o controle;
`Falha` mantém o motivo até novo comando ou zero manual.

Desabilitar o robô remove a sustentação ativa. Se a transmissão mudar,
voltar `ROTATION_REDUCTION_CONFIRMED` para `false`, medir e corrigir a redução.

### Firmware do Spark Flex (giro)

O projeto usa a REVLib 2026.0.5, a mais nova publicada pela REV em 12/11/2026.
Ela exige firmware **2026** no Spark. Com firmware antigo, a Driver Station mostra
`The SPARK firmware version is too new for this version of REVLib ... update the
device to 2026 firmware` e `Invalid parameter id (112) Position Conversion Factor`.

- **Como resolver:** REV Hardware Client 2, USB no Spark 41, atualizar o firmware
  para a versão 2026 e enviar o código de novo.
- **O que o código faz:** se o Spark recusar a configuração, o programa **não cai
  mais**. O giro fica desligado, `Manipulador/Giro/Zero` mostra `FALHA_CONFIG_SPARK`
  e a Driver Station mostra o motivo. Se o manipulador inteiro falhar ao iniciar,
  o pivô continua funcionando.
- **TalonFX ausentes:** com só o Spark na CAN, os erros `-1003`/`-10021` dos TalonFX
  38, 39, 42 e 43 são esperados. Para não criar esses motores no código (e limpar o
  log), use as chaves abaixo.
- **Firmware 27.x é da temporada 2027:** o REV Hardware Client marca a 27.0.0 como
  "Latest", mas ela remove os fatores de conversão. Para este projeto, usar
  **26.1.6** (resolvido assim em 12/11/2026).

### Motores fora da CAN (chaves nas constantes)

| Chave | Arquivo | Motores |
| --- | --- | --- |
| `kEnabled` | `ElevatorPivotConstants.java` | Pivô, TalonFX 38 e 39 |
| `CLAW_MOTOR_ENABLED` | `ManipuladorConstants.java` | Garra, TalonFX 42 |
| `WHEELS_MOTOR_ENABLED` | `ManipuladorConstants.java` | Rodas, TalonFX 43 |
| `ROTATION_MOTOR_ENABLED` | `ManipuladorConstants.java` | Giro, Spark Flex 41 |
| `SWERVE_MOTORS_ENABLED` | `SwerveConstants.java` | Swerve, 8 Spark MAX (IDs 2, 7, 9, 15, 17, 18, 20 e 31) |
| `GYRO_ENABLED` | `SwerveConstants.java` | Pigeon 2 (sem ele, a frente vem das rodas) |

**Em 18/11/2026 o robô está só com a swerve:** as chaves do pivô e do manipulador
estão em `false` (o log fica só com o que existe no robô). As do swerve, em `true`.

Com `false`, o código não cria o motor no robô: aquela
parte nunca se move e o painel mostra `DESLIGADO_NAS_CONSTANTES` (manipulador) ou
`Pivo desligado: ElevatorPivotConstants.kEnabled = false` (pivô). A simulação
ignora as chaves do manipulador. **Voltar para `true`** ao ligar o motor na CAN.

### Referência inicial do manipulador

**Garra:** a equipe confirmou que ela sempre começa aberta pela mola. Por isso,
ela zera automaticamente uma única vez após iniciar o programa, com o robô
ainda desabilitado e comunicação válida. Não é necessário apertar botão para
esse zero. `Manipulador/Garra/Zero` passa de `AGUARDANDO_ZERO_AUTOMATICO` para
`ZERO_AUTOMATICO_OK`.

**Giro:** continua exigindo a referência própria. Com o robô desabilitado e o
giro na marca física de 0°, tocar **BACK** e conferir `ZERO_MANUAL_OK` em
`Manipulador/Giro/Zero`. BACK também permite refazer o zero da garra aberta,
mas não é necessário para a partida normal dela.

> ⚠️ 23/11/2026: **BACK também liga/desliga a velocidade reduzida do swerve** (ver
> "Pilotagem no Xbox"). Com os motores do manipulador desligados em
> `ManipuladorConstants` o zero dele não faz nada; antes de ligar o manipulador, trocar o
> botão de um dos dois (o código avisa na Driver Station se os dois estiverem ligados).
> Enquanto isso, o zero também está no painel: `Manipulador/Zerar`.

Depois de uma perda de referência durante a operação, a garra não assume
uma nova posição como aberta automaticamente. Reposicionar aberta e usar BACK
para recuperar. Se habilitar antes de concluir o zero inicial, o painel mostra
`SEM_ZERO_AUTOMATICO_USE_BACK`. START permanece reservado ao zero do pivô.
Na simulação, os modelos começam em uma posição conhecida.

### Primeiro teste nos motores reais

1. Conferir no Tuner X (CAN 42 e 43) e no REV Hardware Client (CAN 41), com pouca
   saída, os sentidos: garra positivo = fecha, rodas positivo = puxa, giro
   positivo = anti-horário visto de trás.
2. Ligar o robô com a garra aberta pela mola e o manipulador como no desenho.
   A garra zera sozinha: conferir `ZERO_AUTOMATICO_OK` no painel.
   Para o giro, desabilitado e na referência de 0°, tocar BACK e conferir `ZERO_MANUAL_OK`.
3. Habilitar Teleop e testar uma coisa por vez: **→** fecha, **←** solta,
   **↑/↓** segurados giram por posição com MAXMotion. Acompanhar correntes, ângulos e `Manipulador/Garra/Falha`.
4. Passar para a equipe de programação os limites testados (cursos, correntes,
   aperto e tempos) para substituir os provisórios.
Para simular o fechamento vazio, ajuste `SIM_OBJECT_WIDTH_METERS` para `0.0`
em `ManipuladorConstants.java`. O manipulador é instanciado no simulador.

1. Abra **App > Use Custom Assets Folder** no AdvantageScope e selecione
   `config/advantagescope/mecanismos` neste projeto.
2. Em **File > Import Layout**, importe
   `config/advantagescope/layouts/FRC9460-Manipulador.json`.
3. Inicie **WPILib: Simulate Robot Code** no VS Code. No AdvantageScope, use
   **File > Connect to Simulator > NetworkTables 4** e abra **Manipulador 3D**.
4. Na janela **Robot Simulation**, selecione **Teleoperated**. No direcional
   do Xbox na porta 0, segure **→** para pegar e fechar, **←** para soltar,
   **↑** para girar em malha fechada até o fim (95,6°) e **↓** para voltar a 0°. A garra segura o objeto
   até receber a ordem de soltar ou até desabilitar a simulação.

O comando de demonstração `SmartDashboard/Manipulador/TesteCompleto`
executa o ciclo inteiro com a simulação habilitada. A aba **Teste - etapas**
mostra o andamento e o resultado. Os gráficos do layout usam os valores
de garra, rodas e giro publicados por NetworkTables.

## Swerve (MAXSwerve 3")

O subsistema `subsystems/swerve` controla 4 módulos REV MAXSwerve de 3"
(REV-21-3005). Cada módulo tem um Spark MAX + NEO 1.0 na tração e um Spark MAX +
NEO 550 no giro. Os 4 Through Bore Encoders (absolutos) estão ligados **direto no
roboRIO** (DIO 0 a 3, fio branco = sinal absoluto), não nos Spark: o PID do giro roda
no Spark com o encoder do NEO 550, alinhado pelo absoluto ao ligar (detalhes em
"Giro com o encoder no roboRIO"). O giroscópio é um Pigeon 2 (CTRE) no centro do robô. A lógica segue o template
oficial da REV (MAXSwerve Java Template v2026.0) com as boas práticas da WPILib.
No roboRIO usa os motores reais; no computador, a simulação física.

### Motores, IDs e medidas

IDs já gravados no robô (informados pela equipe em 17/11/2026; frente esquerda e
Pigeon corrigidos em 18/11/2026).

| Módulo | Tração (NEO) | Giro (NEO 550) | Encoder absoluto | Tração invertida |
| --- | --- | --- | --- | --- |
| FE — frente esquerda | 9 | 17 | DIO 0 | sim |
| FD — frente direita | 2 | 20 | DIO 1 | não |
| TE — trás esquerda | 7 | 18 | DIO 2 | sim |
| TD — trás direita | 15 | 31 | DIO 3 | não |
| Pigeon 2 (centro) | 12 | — | — | — |

- **DIO:** a equipe informou as portas 0 a 3 (18/11/2026). 🚧 Qual módulo está em qual
  porta ainda não foi conferido (passo 3 de "Antes do primeiro teste").
- **Tração invertida na esquerda:** no teste de 18/11/2026 (rodas retas), duty +0.02
  levou o robô para a frente na direita (2 e 15) e −0.02 na esquerda (9 e 7). Os
  módulos da esquerda estão espelhados, então a tração deles é invertida no código
  (`FE_DRIVE_INVERTED` e `TE_DRIVE_INVERTED`). Mudar "Inverted" só no REV Hardware
  Client não adianta: o código grava a configuração nos Spark toda vez que liga.

| Medida | Valor | De onde veio |
| --- | --- | --- |
| Chassi | 26,5" × 26,5" | Informado pela equipe em 17/11/2026 |
| Entre os eixos das rodas | 23" × 23" (0,584 m) | Informado; (26,5 − 23) / 2 = 1,75" até a borda, como no MAXSwerve |
| Roda | 3" (0,0762 m) | MAXSwerve 3" |
| Redução da tração | 4,71:1 (pinhão de 14 e engrenagem de 22) | Contado pela equipe em 17/11/2026; (45 × 22) / (14 × 15), fórmula REV |
| Velocidade livre | ~4,80 m/s com 14 dentes | NEO 5676 RPM |
| Redução do giro | ~46,42:1 = UltraPlanetary 3:1 e 4:1 × 62/14 | Conferido pela equipe em 17/11/2026 (cartuchos 3:1 e 4:1, engrenagem de 62); converte voltas do NEO 550 em ângulo do módulo |
| Encoder do giro | Through Bore Encoder REV-11-1271 (V1) | Lista de peças do kit REV-21-3005 (🚧 conferir a etiqueta) |

### Kit REV-21-3005: qual pinhão está montado

O kit não tem uma redução fixa: vem com os **três** pinhões do NEO de tração
(12, 13 e 14 dentes) e a engrenagem de 22 dentes, e a equipe escolhe um ao montar.
Em 17/11/2026 a equipe contou: **pinhão de 14 dentes** no NEO, engrenagem de 22
e pinhão cônico de 15 (velocidade alta, sem o kit de upgrade). É o que está em
`SwerveConstants.java`. Se trocarem o pinhão no módulo, trocar também no código.

| Pinhão | Redução | Roda livre (NEO) | `VoltasDoMotor` em 10 voltas da roda |
| --- | --- | --- | --- |
| 12 dentes (baixa) | 5,50:1 | 4,12 m/s | ~55,0 |
| 13 dentes (média) | 5,08:1 | 4,46 m/s | ~50,8 |
| 14 dentes (alta) | 4,71:1 | 4,80 m/s | ~47,1 |

**Descobrir a redução** (sem desmontar):

1. Na caixa do kit devem sobrar dois pinhões: o que falta é o que está no módulo.
2. Ou medir: robô ligado, código rodando e **desabilitado**, rodas fora do chão.
   Marcar a roda com fita, anotar `Swerve/FE/Tracao/VoltasDoMotor`, girar a roda
   **10 voltas** para a frente com a mão e anotar de novo. A diferença dividida
   por 10 é a redução (tabela acima). Esse valor não depende do que está no código.
3. Corrigir `DRIVE_PINION_TEETH` em `SwerveConstants.java`. Com o kit de upgrade
   REV-21-3008 (velocidades extra altas), preencher também `DRIVE_SPUR_TEETH`.
   Os testes recusam uma combinação que não encaixa no módulo.

### Limites dos motores (template REV)

| Limite | Valor |
| --- | --- |
| Corrente da tração (smartCurrentLimit) | 50 A por NEO |
| Corrente do giro | 20 A por NEO 550 (o motor pequeno esquenta rápido) |
| Compensação de tensão | 12 V: saída 1,0 = 12 V, ganhos iguais com qualquer carga da bateria |
| Velocidade máxima de cada roda | 4,80 m/s (desaturação da cinemática) |
| Giro máximo do robô | 360°/s |
| Aceleração pedida pelo piloto | 8 m/s² e 1440°/s² (🚧 ajustar no robô) |

Os Spark ficam em Brake e a configuração é gravada neles ao ligar: se um
reiniciar no meio do jogo, mantém limites, conversões e ganhos. Se o Spark da
tração reiniciar, a odometria ignora o salto do encoder (`Swerve/XX/ReiniciosTracao`).

### Painéis do swerve: AdvantageScope e Elastic (19/11/2026)

Gerados por `python config/gerar_layouts.py` (o script confere no código que cada chave
existe; mudou um nome no código, rode de novo). Robô sem Limelight: só odometria
(rodas + Pigeon 2).

- **AdvantageScope:** `config/advantagescope/layouts/FRC9460-Pendulo.json` (File → Import Layout).
  Um layout só para tudo: robô (File → Connect to Robot), simulação (Simulate Robot Code no
  VS Code e File → Connect to Simulator) e arquivo .wpilog. Sem AdvantageKit, o AdvantageScope
  chama as chaves de `NT:/SmartDashboard/...` ao vivo e no arquivo (19/11: a primeira versão
  usava `/SmartDashboard/...` e as fontes apareciam riscadas).
- Abas: campo 3D e 2D com o robô, as rodas (ciano = medido, laranja = alvo) e o **rastro**
  dos últimos ~15 s (amarelo); robô de perto; aba Swerve; velocidade e giro; X/Y e frente
  (odometria × Pigeon); **giro e tração de cada módulo, alvo × medido** (é aqui que se
  ajusta o PID); correntes; tensões; encoders absolutos; bateria; tabela de status.
- **Modelo 3D:** `FRC9460 - Base Swerve`. O AdvantageScope lê os modelos de
  `FRC9460_Calibracao/config/advantagescope/mecanismos` (App → Use Custom Assets Folder);
  a pasta `Robot_FRC9460_Base_Swerve` foi copiada para lá do FRC Projeto 2027. Sem ela,
  aparece o KitBot no lugar.
- **Elastic:** `src/main/deploy/Elastic-Pendulo-Odometria.json` (cópia igual em
  `config/elastic/`, para abrir no PC: File → Open Layout). Com o robô ligado depois
  do deploy: File → Download From Robot (o código serve a pasta deploy na porta 5800).
  Ou File → Open Layout no arquivo do PC. Simulação: Settings → IP Address Mode → localhost.
  Abas: **Odometria** (campo com o rastro, rodas, X/Y/frente, velocidade, pilotagem (Y),
  velocidade normal/reduzida (BACK), Pigeon, bateria, botão Zerar frente), **Módulos**, **Calibração** (leituras e linhas `OffsetParaCopiar`) e **Saúde**.
- Novas chaves para os painéis: `Swerve/Rastro` (Pose2d[]; também o objeto "Rastro" no
  `Swerve/Campo`), `Swerve/Modulos` (widget SwerveDrive do Elastic), `Robo/BateriaVolts` e
  `Robo/Brownout`.

### Pilotagem no Xbox (porta 0)

Em 19/11/2026 (pedido da equipe), com o pivô desligado (`ElevatorPivotConstants.kEnabled
= false`), **Y, A e START passaram para o swerve**. Antes de ligar o pivô de novo,
trocar os botões de um dos dois (o código avisa na Driver Station se os dois estiverem
ligados). Em 23/11/2026, **BACK** (velocidade reduzida) também passou para o swerve; ele
era o zero do manipulador, que está com os motores desligados.

| Controle | Ação |
| --- | --- |
| Analógico esquerdo | Andar: para cima = para longe da Driver Station (campo) ou para a frente do robô (robô) |
| Analógico direito (X) | Girar o robô (esquerda = anti-horário) |
| **Y** (um toque) | Alterna pilotagem **orientada ao campo** (começa assim) ↔ **orientada ao robô**. Aparece em `Swerve/Pilotagem` e na Driver Station |
| **A** ou LB/L1 segurado | **Freio**: rodas em X, o robô resiste a ser empurrado. Soltar volta a dirigir |
| **START** (um toque, Teleop) | As 4 rodas param e apontam para a frente (0°), sem inverter nenhuma 180°. Mexer um analógico volta a dirigir |
| **BACK** (botão ao lado do START, um toque) | Liga/desliga a **velocidade reduzida**: 30% a menos no andar e no giro (70%). Outro toque volta ao normal. Começa normal, vale desabilitado também e aparece em `Swerve/Velocidade` e na Driver Station |
| LT/L2 segurado | Modo lento (35% da velocidade; com a velocidade reduzida ligada, 0,70 × 0,35 ≈ 25%) |
| L3 (apertar o analógico esquerdo) | Zerar a frente: com o robô apontado para longe da Driver Station |

- **Zona morta 0,12** (era 0,08): nos logs do Xbox da equipe, com o analógico solto ele
  parava em até 0,078 (esquerdo) e 0,094 (direito), e com 0,08 o robô podia andar
  sozinho devagar. A saída começa do 0 na borda da zona morta e é ao quadrado (mais
  precisão devagar).
- **Orientada ao robô** não depende da frente do robô no campo: é a opção enquanto o
  Pigeon estiver sem o firmware 2026 (a frente vem das rodas e escorrega com o tempo).
- **START** serve para conferir o alinhamento com uma barra reta e para deixar as rodas
  retas antes de reiniciar o código (a TD zera ao ligar).

- Vale nas duas alianças: no vermelho, "para a frente" continua sendo para longe
  da sua Driver Station (a origem do campo fica sempre no lado azul, padrão WPILib).
- Só pilota no **Teleop habilitado**. No autônomo, sem outro comando, o robô fica
  parado com as rodas no lugar. Desabilitar coloca os 8 motores em neutro (Brake).
- Sem pedido de movimento por 0,25 s (comando travado), os módulos param
  (`Swerve/Estado` = `SEM_PEDIDO_PARADO`).
- Sem leitura do giro de um módulo, aquele módulo inteiro para (a roda não pode
  tracionar apontando para um lado qualquer); os outros continuam.
- Sem o Pigeon, a frente do robô passa a vir das rodas e continua de onde parou
  quando ele volta. A pose usa `SwerveDrivePoseEstimator`, pronto para visão
  (`addVisionMeasurement`).

### Giro com o encoder no roboRIO

O Through Bore está no roboRIO, então o Spark do giro não enxerga o absoluto. Antes
(até 18/11/2026) o código mandava o Spark usar um encoder absoluto que não estava
nele: **Sensor Fault** nos 4 Spark do giro e nenhum giro. Agora:

1. **Zero ao ligar** (escolha da equipe, 18/11/2026): o robô sempre liga com as 4
   rodas retas para a frente. Com o módulo parado, o NEO 550 vai para 0° e a leitura
   do absoluto nessa hora vira o offset dele (espera até 0,5 s pelo sinal do absoluto).
   O PID do giro roda no Spark (1 kHz) com o encoder do NEO 550 e a redução de
   46,42:1, com os ganhos do template REV.
2. Parado e longe do absoluto mais de 2° (desabilitado) ou 10° (habilitado, ex.: o
   Spark do giro reiniciou), realinha sozinho.
3. A cada 45° que o módulo gira (conferido parado há 100 ms), confere que o absoluto
   andou junto com o NEO 550 (`Swerve/XX/Giro/RazaoAbsolutoPorMotor` ≈ 1,00). Errado 2
   vezes seguidas: sentido invertido, DIO de outro módulo ou absoluto parado **param** o
   módulo; proporção fora (`GIRO_NAO_CONFERE`) só **avisa** e o módulo continua.
   (19/11/2026: era 15° e a proporção fora parava o módulo; a TE parou com razão 1,54 por
   causa do atraso da leitura do Spark pela CAN e da folga, que pesam muito em 15°.)
4. Sem o absoluto (sem sinal ao ligar ou depois), o módulo anda só pelo NEO 550 e
   avisa. Se o sinal voltar, o absoluto herda o zero do NEO 550.
5. Com um offset escrito em `SwerveConstants.java` (diferente de 0.0), o 0° vem do
   absoluto e não precisa ligar com as rodas retas; aí, sem o absoluto, o módulo não
   anda.

### Antes do primeiro teste no robô

1. **IDs e firmware:** no REV Hardware Client 2, conferir os IDs da tabela acima e os
   8 Spark MAX no firmware 2026 (26.x; não usar a 27.x). Pigeon 2 com ID 12 no
   Phoenix Tuner X.
2. **Rodas retas e enviar o código:** antes de ligar o robô (e antes de cada deploy ou
   "Restart Robot Code"), deixar as 4 rodas retas para a frente, na mesma posição do
   teste da tração (sem girar nenhuma 180°). Encostar um perfil de alumínio reto nas
   duas rodas de cada lado ajuda. Ao ligar, a Driver Station mostra para cada módulo:

   ```
   Swerve FE: zerado com a roda reta ao ligar (encoder absoluto 123.45°, DIO 0)
   ```

   ⚠️ Código reiniciado com as rodas tortas = zero errado (as rodas andam tortas pelo
   mesmo ângulo): alinhar e reiniciar o código de novo. Até as 22h de 18/11/2026 o
   código esperava uma calibração com os offsets e, sem ela, deixava a base parada
   ("nenhum motor funcionou").

   Cada `Swerve/XX/EncoderAbsolutoConectado` deve estar verdadeiro e
   `Swerve/XX/EncoderAbsolutoHz` perto de 976 (Through Bore V1). Sem sinal: conferir o
   conector no roboRIO (branco no pino S, do lado de dentro; vermelho no meio; preto na
   borda) e o cabo de 6 fios bem encaixado no encoder, em cima do módulo.
3. **🚧 Conferir as portas e o sentido, com a mão:** robô desabilitado, girar **um**
   módulo devagar no sentido **anti-horário** (visto de cima):
   - só o `Swerve/XX/EncoderAbsolutoGraus` **dele** muda. Se mudar o de outro módulo,
     trocar os números `*_ABSOLUTE_ENCODER_DIO` em `SwerveConstants.java` (ou os cabos);
   - o valor **aumenta**. Se diminuir (nos 4), trocar `TURN_ENCODER_INVERTED` para `false`.
4. **Não depender de ligar com as rodas retas** (feito em 19/11/2026 para FE, FD e TE):
   com as rodas retas, copiar as linhas de `Swerve/XX/OffsetParaCopiar` (`public static
   final double FE_ABSOLUTE_OFFSET_DEG = ...;`, sempre a leitura **atual** do absoluto)
   para `SwerveConstants.java` e enviar o código. Com isso o 0° vem sempre do encoder
   absoluto. Sem sinal do absoluto por 1 s ao ligar, o módulo zera com a roda reta e
   avisa (não fica parado).
   - Por quê: nos logs de 18 e 19/11 o zero ao ligar variou até 4° de uma vez para outra
     (às 13:21 a FE ligou 3,1° torta, a FD 4,4° e a TE 2,3°), e o robô puxava para o lado.
   - 🚧 **Conferir com uma barra reta** encostada nas rodas (robô desabilitado):
     `Swerve/XX/AnguloGraus` entre −1° e +1°. Se não, alinhar com a barra e copiar de novo.
   - **TD sem absoluto** (`TD_ABSOLUTE_ENCODER_DIO = NO_ABSOLUTE_ENCODER`): o DIO 3 ora
     não tinha sinal, ora lia **o mesmo valor da FE** (156,04 × 156,06; 157,40 × 157,42).
     Arrumar o cabo, conferir com a mão (passo 3) e voltar para `3`. O código agora avisa
     em vermelho (e em `Swerve/EncodersAbsolutosRepetidos`) quando dois DIOs leem o mesmo
     encoder enquanto giram.
5. **Robô suspenso**, Teleop, analógico esquerdo um pouco para a frente: as 4 rodas
   apontam para a frente e giram para a frente. Uma roda que gira para **trás** estava
   girada 180° ao ligar: alinhar de novo e reiniciar o código.
   Analógico para a esquerda: as 4 apontam para a esquerda.
6. **Girar o robô com a mão no sentido anti-horário** (visto de cima):
   `Swerve/Giroscopio/YawGraus` e `Swerve/FrenteGraus` devem aumentar.
7. No chão, devagar (LT), conferir: frente, lado, giro e LB (rodas em X). Andar 3 m
   medidos na trena e comparar com `Swerve/PoseXMetros`; se diferir, corrigir
   `WHEEL_DIAMETER_METERS` (a banda gasta diminui o diâmetro).

`TURN_ENCODER_IS_V2`: o kit traz o V1 (REV-11-1271). Se a etiqueta do encoder for
REV-11-3174 (V2), trocar para `true`. O Pigeon 2 já foi conferido (17/11/2026):
deitado, com a seta X para a frente, `GYRO_MOUNT_*` = 0/0/0.

Os ganhos (tração kP = 0,04 e kV = 12 V / 4,80 m/s; giro kP = 1) são os do
template REV. Para o ajuste fino da tração, medir kS e kV no robô (SysId).

### Diagnóstico na Driver Station

Ao ligar, o console mostra os dois Spark e o DIO de cada módulo:

```
Swerve FE: Spark MAX 9 (tração, invertida) firmware 26.1.5 ok | Spark MAX 17 (giro) firmware 26.1.5 ok | encoder absoluto no DIO 0
```

Módulo parado: `Swerve FE: PARADO: <motivo>`, em vermelho (aparece quando muda e de
novo ao habilitar; no painel, `Swerve/Falha` e `Swerve/XX/Configuracao`).

| Motivo | O que fazer |
| --- | --- |
| `SEM_ENCODER_ABSOLUTO (DIO n)` | Só com offset escrito no código: sem sinal desde que ligou. Branco no pino S, vermelho no meio, preto na borda; cabo de 6 fios encaixado no encoder |
| `GIRO_INVERTIDO_OU_ENCODER_TROCADO` | O absoluto andou ao contrário do NEO 550. Conferir o passo 3 (DIO e sentido); se estiver certo nos 4 módulos, trocar `TURN_INVERTED` |
| `ENCODER_ABSOLUTO_NAO_ACOMPANHA` | O NEO 550 girou o módulo e o absoluto não: DIO de outro módulo, encoder solto ou fora do eixo |
| `ENCODER_ABSOLUTO_DE_OUTRO_MODULO` | O absoluto andou e o NEO 550 deste módulo quase nada (menos de 20%): DIO trocado. Até 19/11/2026 este aviso aparecia por engano (FE e TE paravam com razão 1,06 a 1,09, que é normal): corrigido em `SwerveConferenciaGiro` |
| `GIRO_NAO_CONFERE` (aviso, o módulo continua) | Mesmo sentido, proporção fora de 0,7 a 1,4 em 2 janelas de 45°: o módulo segue pelo NEO 550 e realinha parado. Se aparecer sempre no mesmo módulo, conferir pinhão do NEO 550 solto, encoder absoluto folgado no eixo ou a redução do giro (`TURN_REDUCTION`) |
| `FALHA_SENSOR_GIRO (Spark n ...)` | O Spark do giro acusa falha de **sensor** (por 0,1 s): não lê o sensor interno do NEO 550 e o motor não gira. Conferir o cabo fino do encoder do NEO 550 nas duas pontas (pinos tortos, cabo esmagado) e os 3 fios do motor; no REV Hardware Client a falha "Sensor" tem que sumir e a posição tem que mudar girando a roda com a mão. Desde 19/11/2026 o módulo inteiro para (antes a tração seguia com a roda sem direção; foi o que aconteceu com a TE, Spark 18) |
| `DIO_INDISPONIVEL` | Porta DIO inválida ou usada por outra coisa no código |
| `FIRMWARE_TRACAO` / `FIRMWARE_GIRO` | Spark fora da 26.x: atualizar no REV Hardware Client 2 |
| `SEM_LEITURA_TRACAO (Spark 9: kTimeout)` | Spark sem resposta na CAN: ID, cabo CAN, alimentação |

Cada motivo para só aquele módulo; os outros continuam. Os da conferência do giro
somem depois de o módulo girar certo 3 vezes 45° (com a mão, desabilitado) ou com
"Restart Robot Code"; mudou uma constante, enviar o código de novo.

- **Avisos (o módulo continua andando):** falha ativa de um Spark (`sensor`, `can`...,
  só se repetir por 0,1 s) e `encoder absoluto (DIO n): sem sinal, girando só pelo NEO
  550` (conferir o conector: branco no pino S, vermelho no meio, preto na borda).
  Falhas sérias (temperatura, driver) o próprio Spark corta.
- **Teste dos motores pelo REV Hardware Client** (18/11/2026): com o código rodando e
  a Driver Station habilitada, o duty cycle do Hardware Client não funciona ("Robot
  Heartbeat Detected"). Para esse teste, `SWERVE_MOTORS_ENABLED = false`, enviar o
  código, testar e voltar para `true`. Resultado de 18/11/2026: os 4 NEO 550 giram (a
  partir de duty 0.02, no mesmo sentido) e a tração vai para a frente com +0.02 na
  direita (2 e 15) e −0.02 na esquerda (9 e 7).
- **Firmware:** em 18/11/2026 os 8 Spark MAX estavam na **25.0.4** (rodas "pulando",
  giro parado) e foram atualizados para a **26.1.5**. A REVLib 2026 precisa da 26.x; a
  27.x não usa os fatores de conversão.
- **Pigeon 2 com firmware antigo** (`FirmwareVersNotCompatible`, 18/11/2026):
  atualizar no Phoenix Tuner X para a versão 2026 (Phoenix 6 26.x) e reiniciar o
  código. Até lá o robô anda com a frente calculada pelas rodas. O código avisa uma
  vez (`Swerve: Pigeon 2 (CAN 12) com firmware que o Phoenix 6 2026 não aceita`) e
  para de chamar o Pigeon, para os erros -1003/-1008 não esconderem as outras
  mensagens.
- **PDH com Hardware Fault "CSA Channel Read/Invalid"** (visto em 18/11/2026): é o
  chip que mede corrente. As leituras de corrente ficam erradas (71,88 A com o robô
  parado), mas, segundo a REV, a distribuição de energia continua segura. Bolinha
  vermelha num canal no REV Hardware Client = disjuntor desarmado ou ausente.

### Simulação do swerve

Cada módulo simulado tem a física do NEO e do NEO 550 (corrente, tensão, redução,
1/4 da massa do robô, aderência da roda e arrasto do giro no carpete) e emula o
Spark MAX: os mesmos PIDs e feedforward, compensação de tensão, limite de corrente
e Brake quando desabilitado. O Pigeon simulado soma o giro calculado pelas rodas.
Massa (50 kg), aderência e arrasto são estimativas (🚧 em `SwerveConstants.java`).

1. **WPILib: Simulate Robot Code** no VS Code, **Teleoperated** na janela
   Robot Simulation, com o Xbox na porta 0 (o teclado não tem o analógico direito).
2. Na janela da simulação: **NetworkTables > SmartDashboard > Swerve/Campo** mostra
   o robô andando no campo.
3. No AdvantageScope (**File > Connect to Simulator > NetworkTables 4**):
   - aba **Swerve**: arrastar `SmartDashboard/Swerve/EstadosMedidos`,
     `EstadosAlvo` e `Rotacao` (ordem dos módulos FE, FD, TE, TD);
   - aba **2D Field** ou **3D Field**: arrastar `SmartDashboard/Swerve/Pose`;
   - gráficos: `Swerve/XX/VelocidadeMetrosPorSeg`, `AlvoVelocidadeMetrosPorSeg`,
     `Tracao/CorrenteAmps` e `Giro/CorrenteAmps`.

Tudo isso também vai para o `.wpilog` do robô (seção abaixo).

## Log no AdvantageScope

Desde 12/11/2026 o `Robot.java` grava um **`.wpilog`** com tudo o que vai para o
NetworkTables (`Manipulador/*`, `ElevatorPivot/*`, poses do modelo 3D) e com a
Driver Station e o controle (`DataLogManager` + `DriverStation.startDataLog`).
Antes disso não havia log da WPILib: só o `.revlog` da REV e o `.hoot` da CTRE,
sem os dados do manipulador nem do pivô.

- **Onde fica:** com pen drive no roboRIO, em `/u/logs` (recomendado; a REV também
  avisa para não gravar na memória interna). Sem pen drive, em `/home/lvuser/logs`.
  Um arquivo novo por vez que o código inicia.
- **Abrir depois:** AdvantageScope, **File > Download Logs**, conectar no roboRIO
  (USB `172.22.11.2` ou o número da equipe) e baixar o `.wpilog`. Ou abrir o
  arquivo copiado do pen drive.
- **Ao vivo:** **File > Connect to Robot** (no robô real; "Connect to Simulator" é
  só para a simulação no computador).
- **Layout e 3D:** **File > Import Layout** com
  `config/advantagescope/layouts/FRC9460-Manipulador.json` e **App > Use Custom
  Assets Folder** com `config/advantagescope/mecanismos`. Os nomes do layout batem
  com os publicados pelo código (conferido em 12/11/2026).

## Verificação local

No PowerShell, usar o JDK incluído no WPILib 2026:

```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'
.\gradlew.bat build --console=plain
```

Em 13/11/2026, a revisão foi compilada diretamente com Java 17 e as dependências
instaladas do WPILib 2026.2.1, Phoenix 6 26.3.0 e REVLib 2026.0.5. Os testes JUnit
foram executados com HAL/Driver Station simulados: **67 testes passaram, sem
falhas** (41 do pivô e 26 do manipulador). A verificação estática das APIs de
2026 nos arquivos Java alterados não apontou erros nem avisos. O build Gradle completo não
foi validado nesta sessão porque o ambiente impediu a conexão com seu daemon.

Os testes cobrem limites operacionais, paradas, reversão, timeout, falha de
sensores e a sustentação após soltar o direcional. O manipulador usa modelos
físicos simplificados; o pivô usa IO falso para verificar lógica e comandos.
Nenhum teste confirma fiação, sentidos, folga, capacidade mecânica, ganhos ou
sustentação no robô real. Nenhum código foi enviado ao robô.

Em 17/11/2026 o swerve foi compilado com as bibliotecas deste projeto (WPILib
2026.2.1, REVLib 2026.0.5 e Phoenix 6 26.3.0), sem avisos de API obsoleta, e os
**20 testes de `SwerveTest`** passaram com um HAL simulado fora do Windows.
Rodar `.\gradlew.bat build` no computador da equipe para confirmar os 87 testes
(67 anteriores + 20 do swerve). Os testes do swerve cobrem IDs, medidas, redução,
direção orientada ao campo nas duas alianças, giro, rodas em X, zona morta, modo
lento, limite de aceleração e de corrente, desabilitar, falta do Pigeon e módulo
fora da CAN. Não confirmam fiação, sentidos, calibração nem ganhos no robô real.

Em 18/11/2026, 23h (zero ao ligar): **31 testes de `SwerveTest`** passaram, a bancada
de um módulo (zero ao ligar com e sem o absoluto, Spark do giro reiniciando) passou e
uma simulação do robô inteiro (código real + 8 Spark e 4 encoders simulados) mostrou:
ligando com as rodas retas, as 4 vão para 0°, andam a 1 m/s e viram para 90° com o
analógico para a esquerda; ligando tortas, andam tortas pelo mesmo ângulo. A versão
anterior (das 21h56) deixava a base parada sem os offsets: nenhum comando ia para os
motores, como a equipe viu no robô.

Em 18/11/2026 (giro pelos encoders no roboRIO): **32 testes de `SwerveTest`**
passaram (os 20 acima, a tração invertida na esquerda, as portas DIO, a conversão do
NEO 550, o ângulo do absoluto com o offset, a linha de offset com ponto decimal num
computador em português, o absoluto simulado e 6 da conferência do giro). Uma
bancada com Spark e DIO simulados também passou: espera o absoluto ao ligar, alinha,
vai para 90°, passa de 170° para −170° pelo caminho curto, realinha quando o Spark do
giro reinicia, continua sem o absoluto depois de alinhado, para com o sentido
invertido e acusa a falta do absoluto ao ligar (com offset no código). Isso não
substitui o teste no robô (passos de "Antes do primeiro teste").
