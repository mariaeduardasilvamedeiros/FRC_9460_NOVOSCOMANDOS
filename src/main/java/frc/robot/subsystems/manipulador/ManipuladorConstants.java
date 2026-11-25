package frc.robot.subsystems.manipulador;

// #region 📚 IMPORTS

import edu.wpi.first.math.system.plant.DCMotor;

// #endregion

/**
 * Números do manipulador: garra que FECHA por motor e ABRE por mola, rodas
 * que puxam o objeto, e rotação no eixo longo. Medidas do CAD "Montagem
 * geral rev1"; funcionamento definido pela equipe em 03/11/2026.
 *
 * Três motores, um para cada coisa (definido pela equipe em 11/11/2026):
 * - GARRA: Kraken X60 (CAN 42), em cima do manipulador. Fecha os braços. As
 * duas engrenagens de 50 dentes se engrenam: os braços fecham juntos e
 * centralizam o objeto. Soltou: a mola volta a garra para o 0 (aberta).
 * - RODAS: Kraken X60 (CAN 43), em cima, ao lado do da garra. Gira as rodas
 * flexíveis, que puxam o objeto para dentro enquanto a garra fecha. Não
 * soltam o objeto: quem solta é a mola abrindo a garra.
 * - ROTAÇÃO: Spark Flex (CAN 41) com NEO Vortex. Gira o manipulador 90° para
 * um lado e volta (a garra é simétrica: para o outro lado daria a mesma
 * pegada).
 *
 * Os dois X60 estão montados do mesmo jeito (em cima, lado a lado, eixo para
 * baixo): giram no MESMO sentido e usam a mesma inversão (TOP_MOTORS_INVERTED).
 *
 * O objeto ainda não é conhecido: a garra fecha por FORÇA, até o limite do
 * mecanismo ou até travar no objeto. A corrente do motor é a força de aperto,
 * e o ângulo em que ela parou dá a largura do objeto.
 *
 * Parâmetros usados pelo modelo de simulação.
 */
public final class ManipuladorConstants {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    // #region 🛠️ MOTORES (2 TALONFX KRAKEN X60 + 1 SPARK FLEX NEO VORTEX)

    // IDs gravados nos motores (informados pela equipe em 11/11/2026; o 41 foi
    // conferido no REV Hardware Client). No roboRIO o Manipulador usa os motores
    // reais (IOTalonFX e IOSparkFlex); no computador, os modelos simulados.
    public static final int ROTATION_MOTOR_ID = 41; // Spark Flex: gira a garra
    public static final int CLAW_MOTOR_ID = 42; // TalonFX (X60): fecha a garra (a mola abre)
    public static final int WHEELS_MOTOR_ID = 43; // TalonFX (X60): rodas flexíveis puxam o objeto

    // false = o motor não está na CAN (ex.: testando só o giro): no robô o código
    // não cria o motor, essa parte fica parada e o log não enche de erros de CAN.
    // A simulação ignora estas chaves.
    // Em 18/11/2026 o robô está só com a swerve: false. Voltar para true ao religar o manipulador.
    public static final boolean ROTATION_MOTOR_ENABLED = false;
    public static final boolean CLAW_MOTOR_ENABLED = false;
    public static final boolean WHEELS_MOTOR_ENABLED = false;

    // Os dois X60 ficam em cima, lado a lado, com o eixo para baixo: giram no
    // MESMO sentido. Por isso a inversão dos dois sai de um valor só.
    // Garra: positivo = FECHA. Rodas: positivo = PUXA o objeto para dentro.
    // 🚧 Conferir no Tuner X, um motor por vez e com pouca saída. Se os dois
    // fizerem o contrário (abrir e expulsar), troque só TOP_MOTORS_INVERTED.
    public static final boolean TOP_MOTORS_INVERTED = false;
    public static final boolean CLAW_INVERTED = TOP_MOTORS_INVERTED;
    public static final boolean WHEELS_INVERTED = TOP_MOTORS_INVERTED;

    // 🚧 Rotação (Spark Flex): positivo = anti-horário visto de trás do
    // manipulador. Conferir no REV Hardware Client com pouca saída (0,05).
    public static final boolean ROTATION_INVERTED = false;

    // #endregion

    // #region 📏 MEDIDAS - REDUÇÕES

    // 🚧 Garra: pinhão do X60 na engrenagem de 50 dentes (VP 20DP). O pinhão não
    // está no CAD: a distância entre os eixos (3,95 cm) só fecha com 12 dentes.
    public static final double PINION_TEETH = 12.0;
    public static final double GEAR_TEETH = 50.0;
    public static final double CLAW_REDUCTION = GEAR_TEETH / PINION_TEETH;

    // 🚧 Rodas: no CAD, 50/12 até o pivô e polias de Ø 3,85 cm para Ø 4,12 cm
    // até o braço (≈ 4,46). Conferir como o motor das rodas chega nelas.
    public static final double PIVOT_PULLEY_DIAMETER_METERS = 0.0385;
    public static final double ARM_PULLEY_DIAMETER_METERS = 0.0412;
    public static final double WHEELS_REDUCTION = GEAR_TEETH / PINION_TEETH * ARM_PULLEY_DIAMETER_METERS
            / PIVOT_PULLEY_DIAMETER_METERS;

    // Roda compliant: o nome da peça diz 3", mas o CAD mede 10,1 cm.
    public static final double WHEEL_RADIUS_METERS = 0.0505;

    // Rotação: NEO Vortex no Spark Flex, DIRETO no eixo (1:1), como no CAD.
    // Medido pela equipe no REV Hardware Client em 12/11/2026: do 0 até o fim do
    // giro, o encoder andou ~0,25 volta = ~90° com 1:1, como o projeto pede. O
    // valor de partida antigo (25:1) mandaria o motor dar 6,25 voltas.
    public static final double ROTATION_REDUCTION = 1.0;

    // 🛑 O giro REAL só se move com a redução confirmada no mecanismo. Se a
    // transmissão mudar, voltar para false, medir de novo (redução = voltas do
    // encoder para 90° × 4) e corrigir ROTATION_REDUCTION. A simulação ignora.
    public static final boolean ROTATION_REDUCTION_CONFIRMED = true;

    // #endregion

    // #region 📏 MEDIDAS - PESO

    // Manipulador de garra completo, com os três motores: 6 kg e 300 g (informado
    // pela equipe em 06/11/2026). Usado na simulação do giro (ROTATION_MOI_KG_M2).
    public static final double MASS_KG = 6.3;

    // Manipulador inteiro girando no eixo longo: I = m × r².
    // 🚧 Raio de giração estimado em ~0,1 m, não medido no CAD. Dá ~0,063 kg·m².
    // Com o motor direto, a inércia define o kA e a aceleração possível do giro.
    public static final double ROTATION_GYRATION_RADIUS_METERS = 0.10;
    public static final double ROTATION_MOI_KG_M2 = MASS_KG * ROTATION_GYRATION_RADIUS_METERS
            * ROTATION_GYRATION_RADIUS_METERS;

    // #endregion

    // #region 🧭 MEDIDAS - GARRA (REFERENCIAL DO MANIPULADOR, METROS)

    // X = para a frente (para a boca da garra), Y = esquerda, Z = para cima.
    // Medido no CAD pelo config/advantagescope/scripts/preparar_manipulador.py.
    public static final double ROLL_AXIS_HEIGHT_METERS = 0.40;
    public static final double PIVOT_X_METERS = 0.0079;
    public static final double PIVOT_RIGHT_Y_METERS = -0.0318;
    public static final double PIVOT_LEFT_Y_METERS = 0.0317;
    public static final double AXLE_RIGHT_X_METERS = 0.1828;
    public static final double AXLE_RIGHT_Y_METERS = -0.1299;
    public static final double AXLE_LEFT_X_METERS = 0.1889;
    public static final double AXLE_LEFT_Y_METERS = 0.1182;

    // Pivô até o eixo das rodas (os dois braços medem 20,05 cm).
    public static final double ARM_LENGTH_METERS = 0.2005;
    public static final double PIVOT_HALF_SPACING_METERS = (PIVOT_LEFT_Y_METERS - PIVOT_RIGHT_Y_METERS) / 2.0;

    // Abertura de cada braço no desenho (graus a partir do eixo da garra).
    // Os dois lados estão um pouco diferentes no CAD: 29,3° e 25,5°.
    public static final double CAD_RIGHT_ARM_DEGREES = Math.toDegrees(Math.atan2(
            PIVOT_RIGHT_Y_METERS - AXLE_RIGHT_Y_METERS, AXLE_RIGHT_X_METERS - PIVOT_X_METERS));
    public static final double CAD_LEFT_ARM_DEGREES = Math.toDegrees(Math.atan2(
            AXLE_LEFT_Y_METERS - PIVOT_LEFT_Y_METERS, AXLE_LEFT_X_METERS - PIVOT_X_METERS));

    // 🚧 0 da garra = TODA ABERTA, onde a mola segura: 40° em cada braço, ~22 cm
    // entre as rodas (no desenho ela está a ~27°, 14,7 cm). O CAD não tem batente
    // de abertura: os braços giram livres até 45° sem encostar em nada (conferido
    // girando o modelo). Medir o batente real com a mola montada.
    public static final double CLAW_OPEN_ARM_DEGREES = 40.0;

    // As rodas dos dois braços se encostam com este ângulo (~5,4°).
    public static final double WHEELS_TOUCH_ARM_DEGREES = Math.toDegrees(Math.asin(
            (WHEEL_RADIUS_METERS - PIVOT_HALF_SPACING_METERS) / ARM_LENGTH_METERS));

    // 🛑 Limite do mecanismo: para com ~1,1 cm entre as rodas, sem esmagá-las.
    public static final double CLAW_MIN_ARM_DEGREES = 7.0;

    // Quanto a roda compliant afunda no objeto (para estimar a largura dele).
    public static final double WHEEL_SQUISH_METERS = 0.015;

    // #endregion

    // #region 🛑🚧 SEGURANÇA - LIMITES DE CORRENTE (PROVISÓRIOS, DA SIMULAÇÃO)

    // 🚧 PROVISÓRIOS: tirados da simulação em 12/11/2026, com folga, para os
    // primeiros testes nos motores reais. Trocar pelos valores testados.

    // Garra: o estator é a FORÇA DE APERTO (a garra fecha até travar). Na
    // simulação ela fecha vazia com pico de 15,5 A; 25 A dá ~60% de folga e
    // aperta menos que os 40 A de antes (~5 N por lado na linha das rodas,
    // menos a mola).
    public static final double CLAW_SUPPLY_CURRENT_LIMIT_AMPS = 25.0;
    public static final double CLAW_STATOR_CURRENT_LIMIT_AMPS = 25.0;

    // Rodas: sem carga a simulação pede só 2,5 A (ela não simula o atrito do
    // objeto). 40 A de estator (antes 60) ainda puxa com folga e esquenta menos
    // com as rodas travadas no objeto em WHEELS_PULL_AFTER_CLOSE_SECONDS.
    public static final double WHEELS_SUPPLY_CURRENT_LIMIT_AMPS = 30.0;
    public static final double WHEELS_STATOR_CURRENT_LIMIT_AMPS = 40.0;

    // Rotação: Spark Flex, um limite só (smartCurrentLimit). Com o motor direto
    // (1:1), só a carga medida (15% de saída para girar, ~0,54 N·m) já pede ~32 A
    // parado no fim. A equipe pediu mais força: com o motor quase parado, a força é
    // a corrente (60 A já cortava acima de ~30% de duty). 80 A = padrão do Spark
    // Flex, ~1,36 N·m no manipulador.
    public static final int ROTATION_CURRENT_LIMIT_AMPS = 80;

    // #endregion

    // #region 🛑🚧 SEGURANÇA - TEMPOS MÁXIMOS (PROVISÓRIOS, DA SIMULAÇÃO)

    // Na simulação a garra fecha em 0,28 s (vazia) a 0,36 s (com objeto).
    // Fechando por mais que isto, sem prender nem chegar ao limite: algo travou.
    // Solta a garra (a mola abre); apertar → de novo tenta outra vez.
    public static final double CLAW_CLOSE_TIMEOUT_SECONDS = 1.0;

    // 0° → 90,6° leva ~1,2 s na simulação (também com a carga medida de 15%).
    // Sem chegar neste tempo (fiação presa, motor travado, desbalanço maior que o
    // motor segura): o giro para até soltar o botão.
    public static final double ROTATION_MOVE_TIMEOUT_SECONDS = 3.0;

    // #endregion

    // #region 🛑 SEGURANÇA - CURSOS

    // Garra: do 0 (toda aberta pela mola) até o limite, em graus de fechamento e
    // em voltas do rotor (soft limit do TalonFX). São 33°: ~0,38 volta do rotor.
    public static final double CLAW_CLOSED_LIMIT_DEGREES = CLAW_OPEN_ARM_DEGREES - CLAW_MIN_ARM_DEGREES;
    public static final double CLAW_CLOSED_LIMIT_MOTOR_ROTATIONS = CLAW_CLOSED_LIMIT_DEGREES / 360.0 * CLAW_REDUCTION;

    // 🛑 Soft limit de fechar do TalonFX: 1° além da meta, ainda antes das rodas se
    // encostarem (~34,6°). O Motion Magic para na meta; o soft limit só age se a
    // garra passar dela (antes ele ficava em cima da meta e brigava com a mola).
    public static final double CLAW_SOFT_LIMIT_MARGIN_DEGREES = 1.0;
    public static final double CLAW_CLOSED_SOFT_LIMIT_MOTOR_ROTATIONS =
            (CLAW_CLOSED_LIMIT_DEGREES + CLAW_SOFT_LIMIT_MARGIN_DEGREES) / 360.0 * CLAW_REDUCTION;

    // O motor nunca abre a garra (quem abre é a mola): o soft limit de abrir fica
    // logo atrás do 0.
    public static final double CLAW_OPEN_LIMIT_MOTOR_ROTATIONS = -0.02;

    // Rotação: 0° = como no desenho; gira até o fim medido e volta ao 0.
    // Medido pela equipe (REV Hardware Client, 12/11/2026), girando com a mão:
    // encoder em 0,02846 na posição 0 e em 0,280134 no fim. O curso é a diferença,
    // 0,251674 volta = 90,6° (o código zera o encoder na posição 0, com BACK).
    public static final double ROTATION_MEASURED_ZERO_ENCODER_ROTATIONS = 0.02846;
    public static final double ROTATION_MEASURED_END_ENCODER_ROTATIONS = 0.280134;
    public static final double ROTATION_MEASURED_END_MOTOR_ROTATIONS =
            ROTATION_MEASURED_END_ENCODER_ROTATIONS - ROTATION_MEASURED_ZERO_ENCODER_ROTATIONS;

    // A equipe pediu para ir um pouco além do fim medido (90,6°): +5° = 95,6°.
    public static final double ROTATION_END_EXTRA_DEGREES = 5.0;
    public static final double ROTATION_END_DEGREES =
            ROTATION_MEASURED_END_MOTOR_ROTATIONS / ROTATION_REDUCTION * 360.0 + ROTATION_END_EXTRA_DEGREES;

    // 🛑 Soft limits: 5° de margem além do 0 e além do fim medido (fiação).
    public static final double ROTATION_SOFT_LIMIT_MARGIN_DEGREES = 5.0;
    public static final double ROTATION_MIN_DEGREES = -ROTATION_SOFT_LIMIT_MARGIN_DEGREES;
    public static final double ROTATION_MAX_DEGREES = ROTATION_END_DEGREES + ROTATION_SOFT_LIMIT_MARGIN_DEGREES;
    public static final double ROTATION_MIN_MOTOR_ROTATIONS = ROTATION_MIN_DEGREES / 360.0 * ROTATION_REDUCTION;
    public static final double ROTATION_MAX_MOTOR_ROTATIONS = ROTATION_MAX_DEGREES / 360.0 * ROTATION_REDUCTION;

    // 🛑 Leitura mais velha que isto: as rodas param.
    public static final double MAX_SIGNAL_AGE_SECONDS = 0.10;

    // 🛑 MotorSafety: o código parou de mandar pedido, as rodas param (como a
    // centrífuga).
    public static final double MOTOR_SAFETY_SECONDS = 0.25;

    // #endregion

    // #region 🚧 CONTROLE - GARRA (MOTION MAGIC ATÉ O LIMITE, NO TALONFX)

    // A meta é sempre o limite: sem objeto, a garra para nele. Em voltas do ROTOR.
    // 🚧 Ganhos de partida: com só 4,17:1 os braços são leves e rápidos; o kD
    // amortece (sem ele a garra oscilava ao chegar no limite).
    public static final double CLAW_KP = 15.0;
    public static final double CLAW_KD = 0.3;
    public static final double CLAW_KS = 0.25;
    public static final double CLAW_KV = 12.0 / (6000.0 / 60.0);

    // 🚀 2 voltas/s do rotor = ~170°/s nos braços: fecha em ~0,2 s.
    public static final double CLAW_CRUISE_ROTATIONS_PER_SEC = 2.0;
    public static final double CLAW_ACCELERATION_ROTATIONS_PER_SEC2 = 20.0;

    // Garra parada (mais devagar que isto, por este tempo) antes do limite = objeto
    // preso.
    public static final double CLAW_STOPPED_DEGREES_PER_SEC = 5.0;
    public static final double CLAW_STOPPED_SECONDS = 0.15;

    // Dentro disto do 0 está aberta; dentro disto do limite, fechou vazia.
    public static final double CLAW_POSITION_TOLERANCE_DEGREES = 1.0;

    // 🚧 Objeto preso: segura com tensão fixa. Travado, a corrente é V/R (1 V ≈ 30
    // A), a mesma força para qualquer tamanho de objeto. PROVISÓRIO: 0,6 V ≈ 18 A,
    // abaixo do limite de 25 A da garra (com 1 V o limite cortaria o aperto).
    // Calibrar o aperto no robô junto com CLAW_STATOR_CURRENT_LIMIT_AMPS.
    public static final double CLAW_HOLD_VOLTS = 0.6;

    // #endregion

    // #region 🎯 CONTROLE - RODAS (VELOCITYVOLTAGE NO TALONFX)

    // Em voltas do ROTOR por segundo, como o Phoenix 6. Tensão = kS + kV × alvo +
    // kP × erro.
    public static final double KRAKEN_X60_FREE_SPEED_RPM = 6000.0;
    public static final double WHEELS_KP = 0.10;
    public static final double WHEELS_KS = 0.15;
    public static final double WHEELS_KV = 12.0 / (KRAKEN_X60_FREE_SPEED_RPM / 60.0);

    // 🚀 Rampa da partida: 0 a 3000 RPM em ~0,33 s.
    public static final double WHEELS_ACCELERATION_ROTATIONS_PER_SEC2 = 150.0;

    // 🚀 RPM do rotor: ~3,6 m/s na borda das rodas.
    public static final double INTAKE_RPM = 3000.0;

    // 🚧 Depois que a garra fecha, as rodas seguem puxando para assentar o objeto.
    public static final double WHEELS_PULL_AFTER_CLOSE_SECONDS = 2.0;
    public static final double WHEELS_MAX_RPM = 5500.0;

    // #endregion

    // #region 🎯 CONTROLE - ROTAÇÃO (MAXMOTION NO SPARK FLEX)

    // Em voltas do ROTOR (o encoder do Vortex); com 1:1, uma volta do rotor é
    // uma volta do manipulador. O Spark compensa a tensão para 12 V: saída 1,0 =
    // 12 V. 🚧 Ganhos de partida para o motor direto, ajustados na simulação.
    public static final double ROTATION_NOMINAL_VOLTS = 12.0;
    public static final double NEO_VORTEX_FREE_SPEED_RPM = 6784.0;
    public static final double NEO_VORTEX_STALL_TORQUE_NM = 3.6;

    // Medido pela equipe no REV Hardware Client (12/11/2026), em duty cycle:
    // com 0,01 o manipulador sai da inércia; com 0,15 consegue girar o curso todo.
    // Sair com 1% e precisar de 15% para girar = carga que cresce com o ângulo
    // (peso fora do eixo ou fiação): ~0,54 N·m, ~32 A com o motor parado.
    public static final double ROTATION_BREAKAWAY_DUTY_CYCLE = 0.01;
    public static final double ROTATION_TURN_DUTY_CYCLE = 0.15;

    // kP em fração da saída por volta de erro: 60 = ~2 V por grau. Precisa ser
    // forte: parado no fim, a carga pede 1,8 V; com kP 30 o giro ficava ~1,8° antes
    // do fim e levava ~5 s para entrar na tolerância. Com 60 chega em ~1,2 s, a
    // menos de 1°, também com o dobro da inércia. 🚧 Se oscilar no robô, baixar.
    public static final double ROTATION_KP = 60.0;

    // kS, kV e kA em volts (feedForward da REVLib 2026), seguindo o perfil do
    // MAXMotion. kS = o duty cycle que tira da inércia (0,01 × 12 V). Com o motor
    // direto, o kA pesa muito: V = I·R, com I = inércia × aceleração /
    // (Kt × redução²) em voltas do rotor. Para o Vortex, R/Kt = 12 V / 3,6 N·m.
    // Dá ~1,32 V por volta/s².
    public static final double ROTATION_KS = ROTATION_BREAKAWAY_DUTY_CYCLE * 12.0;
    public static final double ROTATION_KV = ROTATION_NOMINAL_VOLTS / (NEO_VORTEX_FREE_SPEED_RPM / 60.0);
    public static final double ROTATION_KA = ROTATION_MOI_KG_M2 * 2.0 * Math.PI
            * (ROTATION_NOMINAL_VOLTS / NEO_VORTEX_STALL_TORQUE_NM) / (ROTATION_REDUCTION * ROTATION_REDUCTION);

    // 🚀 0,35 volta/s = 126°/s; 0,6 volta/s² = 216°/s². Mais rápido que isso, com
    // o dobro da inércia estimada, o motor (40 A) não freia a tempo na simulação.
    // 0° → 90,6° em ~1,2 s.
    public static final double ROTATION_CRUISE_ROTATIONS_PER_SEC = 0.35;
    public static final double ROTATION_ACCELERATION_ROTATIONS_PER_SEC2 = 0.6;
    public static final double ROTATION_TOLERANCE_DEGREES = 2.0;

    // Meta do giro por posição (MAXMotion), usada no teste completo.
    public static final double ROTATION_TURNED_DEGREES = ROTATION_END_DEGREES;

    // Teto de saida do controlador de posicao; velocidade e aceleracao sao ajustadas acima.
    // Preserva os 60% anteriores, agora para movimento e sustentacao em malha fechada.
    public static final double ROTATION_MAX_OUTPUT = 0.60;
    public static final double ROTATION_STOPPED_DEGREES_PER_SEC = 5.0;

    // #endregion

    // #region 🛠️ TESTE DO MECANISMO (SEQUÊNCIA AUTOMÁTICA)

    public static final double TEST_ROTATION_DEGREES = ROTATION_TURNED_DEGREES;
    // Acima do tempo máximo do giro (ROTATION_MOVE_TIMEOUT_SECONDS).
    public static final double TEST_ROTATION_TIMEOUT_SECONDS = 3.5;

    // Fecha e as rodas ainda puxam WHEELS_PULL_AFTER_CLOSE_SECONDS.
    public static final double TEST_CLOSE_TIMEOUT_SECONDS = 3.0;
    public static final double TEST_HOLD_SECONDS = 0.5;
    public static final double TEST_OPEN_TIMEOUT_SECONDS = 1.0;

    // #endregion

    // #region 🧪 SIMULAÇÃO (ESTIMATIVAS)

    // Esfera do modelo 3D: 15 cm. Use 0.0 para simular a garra vazia.
    public static final double SIM_OBJECT_WIDTH_METERS = 0.15;

    public static final DCMotor CLAW_SIM_MOTOR = DCMotor.getKrakenX60(1);

    // Os dois braços com as rodas, vistos no eixo das engrenagens.
    public static final double CLAW_MOI_KG_M2 = 0.01;

    // 🚧 Mola (ainda não escolhida): segura a garra aberta no batente e
    // aumenta conforme fecha. Torque somado nos dois braços.
    public static final double SPRING_PRELOAD_NM = 0.3;
    public static final double SPRING_RATE_NM_PER_DEGREE = 0.02;

    public static final DCMotor WHEELS_SIM_MOTOR = DCMotor.getKrakenX60(1);

    // 4 rodas compliant, eixos, polias e correias, no eixo das rodas.
    public static final double WHEELS_MOI_KG_M2 = 0.001;

    // Inércia do giro: ROTATION_MOI_KG_M2, em MEDIDAS - PESO.
    public static final DCMotor ROTATION_SIM_MOTOR = DCMotor.getNeoVortex(1);

    // Carga do giro medida (ROTATION_TURN_DUTY_CYCLE): 15% do torque travado do
    // Vortex no fim do curso, puxando de volta para o 0 como um peso fora do eixo
    // (cresce com o seno do ângulo). 🚧 A forma da carga é estimada.
    public static final double ROTATION_SIM_LOAD_AT_END_NM =
            ROTATION_TURN_DUTY_CYCLE * NEO_VORTEX_STALL_TORQUE_NM * ROTATION_REDUCTION;

    // Batente físico da simulação, além do soft limit (a fiação).
    public static final double ROTATION_HARD_STOP_MARGIN_DEGREES = 5.0;

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region ⚙️ CONSTRUÇÃO

    private ManipuladorConstants() {
    }

    // #endregion

    // #endregion
}
