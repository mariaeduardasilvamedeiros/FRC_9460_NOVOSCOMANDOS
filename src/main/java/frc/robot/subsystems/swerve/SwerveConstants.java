package frc.robot.subsystems.swerve;

// #region 📚 IMPORTS

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

// #endregion

/**
 * Números do swerve: 4 módulos REV MAXSwerve de 3" (REV-21-3005). Cada módulo tem
 * 2 Spark MAX: NEO 1.0 na tração e NEO 550 no giro, com o Through Bore Encoder
 * (absoluto) no eixo do giro. Neste robô os 4 encoders estão ligados direto no
 * roboRIO (DIO 0 a 3, fio branco = sinal absoluto), não nos Spark. Giroscópio:
 * Pigeon 2 (CTRE) no centro do robô.
 *
 * Chassi de 26,5" × 26,5" e 23" entre os eixos das rodas nos dois sentidos
 * (informado pela equipe em 17/11/2026): (26,5 − 23) / 2 = 1,75" da borda do
 * chassi até o centro de cada roda.
 *
 * Convenção da WPILib: X = para a frente do robô, Y = para a esquerda, ângulo
 * positivo = anti-horário visto de cima. Ordem dos módulos em tudo (cinemática,
 * arrays e telemetria): FE, FD, TE, TD.
 * FE = frente esquerda, FD = frente direita, TE = trás esquerda, TD = trás direita.
 *
 * Valores de partida do template oficial da REV (MAXSwerve Java Template v2026.0).
 * 🚧 marca o que precisa ser conferido no robô.
 */
public final class SwerveConstants {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    // #region 🛠️ MOTORES E SENSORES (8 SPARK MAX + 1 PIGEON 2)

    // IDs já gravados no robô (informados pela equipe em 17/11/2026; frente esquerda
    // e Pigeon corrigidos em 18/11/2026).
    public static final double SPEED_DEFAULT = 0.5, SPEED_FAST = 0.8, SPEED_SLOW = 0.3; // Velocidade padrão/baixa/alta
    public static final int FE_DRIVE_ID = 9; // Spark MAX + NEO: tração frente esquerda
    public static final int FE_TURN_ID = 17; // Spark MAX + NEO 550: giro frente esquerda
    public static final int FD_DRIVE_ID = 2; // tração frente direita
    public static final int FD_TURN_ID = 20; // giro frente direita
    public static final int TE_DRIVE_ID = 7; // tração trás esquerda
    public static final int TE_TURN_ID = 18; // giro trás esquerda
    public static final int TD_DRIVE_ID = 15; // tração trás direita
    public static final int TD_TURN_ID = 31; // giro trás direita
    public static final int GYRO_ID = 12; // Pigeon 2, no barramento CAN do roboRIO

    // false = os 8 Spark MAX não estão na CAN: o código não cria os motores, o
    // swerve fica parado e o painel mostra DESLIGADO_NAS_CONSTANTES.
    // A simulação ignora estas chaves.
    public static final boolean SWERVE_MOTORS_ENABLED = true;
    // false = sem Pigeon 2 na CAN: a frente do robô vem só das rodas (cinemática),
    // que escorrega com o tempo. O swerve continua andando.
    public static final boolean GYRO_ENABLED = true;

    // Tração: positivo = roda para a frente com o módulo em 0° (apontado para a frente).
    // Teste de 18/11/2026 no REV Hardware Client, com as rodas retas para a frente:
    // duty +0.02 levou o robô para a frente na direita (2 e 15) e −0.02 na esquerda
    // (9 e 7). Os módulos da esquerda estão espelhados em relação aos da direita, então
    // a tração da esquerda é invertida. A calibração dos offsets (abaixo) tem que ser
    // feita com as rodas nessa mesma posição, sem girar nenhuma 180°.
    // O código grava isto nos Spark ao ligar: mudar só no REV Hardware Client não adianta.
    public static final boolean FE_DRIVE_INVERTED = true; // Spark 9
    public static final boolean FD_DRIVE_INVERTED = false; // Spark 2
    public static final boolean TE_DRIVE_INVERTED = true; // Spark 7
    public static final boolean TD_DRIVE_INVERTED = false; // Spark 15

    // NEO 550 não invertido (template REV): os 4 giram no mesmo sentido com duty
    // positivo (teste de 18/11/2026). Se a Driver Station acusar
    // GIRO_INVERTIDO_OU_ENCODER_TROCADO nos 4 módulos depois de conferir o encoder
    // (passo da mão no README), trocar para true.
    public static final boolean TURN_INVERTED = false;

    // No MAXSwerve o eixo de saída do giro gira ao contrário do NEO 550: o encoder
    // absoluto é invertido para que positivo = anti-horário visto de cima (template REV).
    // 🚧 Conferir: girando um módulo com a mão no sentido anti-horário (visto de cima),
    // Swerve/XX/EncoderAbsolutoGraus tem que AUMENTAR. Se diminuir, trocar para false.
    public static final boolean TURN_ENCODER_INVERTED = true;

    // O kit REV-21-3005 (lista de peças da REV) traz o Through Bore Encoder REV-11-1271,
    // o V1. O template REV 2026 usa o V2 (REV-11-3174), vendido à parte.
    // 🚧 Conferir a etiqueta do encoder: se for REV-11-3174, trocar para true.
    // Só muda a leitura do pulso no roboRIO: o V1 pulsa a 975,6 Hz (1 a 1024 µs em 1025 µs).
    // Swerve/XX/EncoderAbsolutoHz mostra a frequência medida (~976 no V1).
    public static final boolean TURN_ENCODER_IS_V2 = false;

    // Encoders absolutos no roboRIO (informado pela equipe em 18/11/2026: DIO 0, 1, 2 e 3).
    // 🚧 Qual módulo está em qual porta não foi conferido. Robô desabilitado: girar UM
    // módulo com a mão e ver que só o Swerve/XX/EncoderAbsolutoGraus dele muda. Se mudar
    // o de outro módulo, trocar os números aqui (ou os cabos).
    // NO_ABSOLUTE_ENCODER (−1) = módulo sem encoder absoluto: o código nem abre o DIO e o
    // giro anda só pelo NEO 550, zerado com a roda reta ao ligar (offset tem que ser 0.0).
    public static final int NO_ABSOLUTE_ENCODER = -1;
    public static final int FE_ABSOLUTE_ENCODER_DIO = 0;
    public static final int FD_ABSOLUTE_ENCODER_DIO = 1;
    public static final int TE_ABSOLUTE_ENCODER_DIO = 2;
    // 🛑 TD desligado do DIO 3 (logs da Driver Station de 18 e 19/11/2026): em metade das
    // vezes que o robô ligou o DIO 3 não teve sinal, e nas outras ele leu EXATAMENTE o
    // encoder da FE (DIO 3 × DIO 0: 156,07 × 156,07; 156,04 × 156,06; 157,40 × 157,42). Ou seja, o fio
    // do DIO 3 está pegando o sinal da FE (pino S encostado/emendado no do DIO 0, ou cabo
    // da FE ligado nas duas portas). Usando o DIO 3, a TD seria "realinhada" pelo ângulo da FE.
    // Para voltar: arrumar o cabo da TD, robô desabilitado girar SÓ a TD com a mão e ver
    // que só Swerve/TD/EncoderAbsolutoGraus muda (e o da FE não); aí trocar para 3.
    public static final int TD_ABSOLUTE_ENCODER_DIO = NO_ABSOLUTE_ENCODER;

    // Pigeon 2 no centro do robô, deitado e com a seta X da etiqueta apontando para a
    // frente (posição confirmada pela equipe em 17/11/2026): 0/0/0.
    // Se mudar a montagem: preencher em graus (Phoenix Tuner X > Pigeon 2 > Mount Pose).
    public static final double GYRO_MOUNT_YAW_DEGREES = 0.0;
    public static final double GYRO_MOUNT_PITCH_DEGREES = 0.0;
    public static final double GYRO_MOUNT_ROLL_DEGREES = 0.0;

    // Yaw e velocidade de giro a 100 Hz; o resto do Pigeon é desligado para aliviar a CAN.
    public static final double GYRO_SIGNAL_HZ = 100.0;

    // #endregion

    // #region 📏 MEDIDAS - CHASSI E POSIÇÃO DOS MÓDULOS

    public static final double FRAME_SIZE_METERS = Units.inchesToMeters(26.5);

    // Centro a centro das rodas: frente-trás (wheel base) e esquerda-direita (track width).
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(23.0);
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(23.0);

    // Posição de cada roda a partir do centro do robô (X frente, Y esquerda).
    public static final Translation2d FE_LOCATION = new Translation2d(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d FD_LOCATION = new Translation2d(WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d TE_LOCATION = new Translation2d(-WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d TD_LOCATION = new Translation2d(-WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);

    // Do centro do robô até cada roda: ~0,413 m.
    public static final double DRIVE_BASE_RADIUS_METERS = Math.hypot(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);

    // 0.0 = ZERO AO LIGAR (escolha da equipe em 18/11/2026: o robô sempre liga com as 4
    // rodas retas para a frente, na mesma posição do teste da tração, sem girar nenhuma
    // 180°). Cada módulo vira 0° quando o código liga e a leitura do absoluto nessa hora
    // vira o offset dele (sai na Driver Station: "Swerve FE: zerado com a roda reta ao
    // ligar (encoder absoluto 123.45°, DIO 0)").
    // ⚠️ Código reiniciado com as rodas tortas (Restart Robot Code, deploy, roboRIO
    // reiniciando) = zero errado: alinhar as rodas e reiniciar o código de novo.
    // Opcional, para não depender disso: com as rodas retas, copiar aqui a linha de
    // Swerve/XX/OffsetParaCopiar ("public static final double FE_ABSOLUTE_OFFSET_DEG = ...;").
    // Número diferente de 0.0 = o 0° vem sempre do encoder absoluto, ligando torto ou não.
    // Sem sinal do absoluto por 1 s ao ligar: o módulo zera com a roda reta (como no zero
    // ao ligar), avisa, e o offset daqui volta a valer quando o sinal aparecer.
    //
    // 19/11/2026: FE, FD e TE passaram a usar offset fixo. Nos logs da Driver Station o zero
    // ao ligar variou de uma vez para outra (a roda nunca liga exatamente reta):
    //   FE: 156,07 / 156,05 / 156,07 / 156,06 / 159,17 / 157,41
    //   FD:  51,98 /  51,90 /  51,99 /  52,30 /  47,59 /  50,74
    //   TE:  27,71 /  27,67 /  36,64 /  28,65 /  25,35 /  30,76
    // Na sessão das 13:21 a FE ficou 3,1° torta, a FD 4,4° e a TE 2,3°: o robô puxa para o
    // lado e as rodas brigam entre si. Os números abaixo são a média das calibrações de
    // 18/11/2026 (21:57 e 21:59, rodas retas pelo procedimento), que bateram em ±0,05°.
    // 🚧 Conferir com uma régua/barra reta encostada nas rodas: robô desabilitado,
    // Swerve/XX/AnguloGraus tem que ficar entre −1° e +1°. Se não ficar, alinhar com a barra
    // e copiar de novo a linha de Swerve/XX/OffsetParaCopiar (mostra a leitura atual).
    public static final double FE_ABSOLUTE_OFFSET_DEG = 156.06; // DIO 0
    public static final double FD_ABSOLUTE_OFFSET_DEG = 51.96; // DIO 1
    public static final double TE_ABSOLUTE_OFFSET_DEG = 27.69; // DIO 2
    // TD sem encoder absoluto (ver TD_ABSOLUTE_ENCODER_DIO): continua no zero ao ligar.
    public static final double TD_ABSOLUTE_OFFSET_DEG = 0.0;

    // #endregion

    // #region 📏 MEDIDAS - RODA E REDUÇÕES (MAXSWERVE 3")

    // 🚧 Roda de 3". Com o desgaste da banda o diâmetro diminui: andar 3 m medidos
    // na trena, comparar com Swerve/PoseXMetros e corrigir aqui.
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.0);
    public static final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_METERS / 2.0;

    // O kit REV-21-3005 vem com os TRÊS pinhões do NEO de tração (12, 13 e 14 dentes)
    // e a engrenagem de 22 dentes: a equipe escolhe UM pinhão ao montar. Os outros
    // dois ficam guardados: o que falta na caixa do kit é o que está no módulo.
    //
    //   Pinhão | Engrenagem | Redução | Roda livre (NEO)
    //   12     | 22         | 5,50:1  | 4,12 m/s  (baixa)
    //   13     | 22         | 5,08:1  | 4,46 m/s  (média)
    //   14     | 22         | 4,71:1  | 4,80 m/s  (alta; o template REV usa esta)
    //
    // Kit de upgrade REV-21-3008 (velocidades extra altas, vendido à parte): pinhão
    // 14, 15 ou 16 e engrenagem de 21, 20 ou 19 dentes. Preencher os dois números.
    // Contados pela equipe em 17/11/2026: pinhão de 14 dentes no NEO e engrenagem de 22
    // (velocidade alta, sem o kit de upgrade): 4,71:1 e ~4,80 m/s com a roda livre.
    // Trocar o pinhão no módulo = trocar este número.
    public static final int DRIVE_PINION_TEETH = 14;
    public static final int DRIVE_SPUR_TEETH = 22;

    // Coroa cônica de 45 dentes na roda e pinhão cônico de 15 dentes (REV; o de 15
    // contado pela equipe em 17/11/2026).
    public static final double DRIVE_REDUCTION = driveReduction(DRIVE_PINION_TEETH, DRIVE_SPUR_TEETH);

    // NEO 550 até o eixo do giro: duas reduções da UltraPlanetary e a engrenagem do giro
    // (conferido pela equipe em 17/11/2026: cartuchos 3:1 e 4:1 no NEO 550 e engrenagem de 62).
    // Cartucho "3:1" = 84:29 (2,90), cartucho "4:1" = 76:21 (3,62), pinhão de 14 dentes na
    // engrenagem de 62 (4,43). Total 9424:203 = 46,42:1, o mesmo número da REV.
    // O PID do giro roda no Spark com o encoder do NEO 550, então esta redução converte
    // voltas do motor em ângulo do módulo. Swerve/XX/Giro/RazaoAbsolutoPorMotor confere:
    // deve ficar perto de 1,00 depois de o módulo girar.
    public static final double TURN_REDUCTION = (84.0 / 29.0) * (76.0 / 21.0) * (62.0 / 14.0);

    // NEO 1.0 sem carga (REV). Roda livre: tabela acima.
    public static final double NEO_FREE_SPEED_RPM = 5676.0;
    public static final double DRIVE_FREE_SPEED_MPS =
            NEO_FREE_SPEED_RPM / 60.0 * Math.PI * WHEEL_DIAMETER_METERS / DRIVE_REDUCTION;

    // Fatores de conversão gravados no Spark: tração em metros e m/s da roda; giro (encoder
    // do NEO 550) em radianos e rad/s do eixo do módulo.
    public static final double DRIVE_POSITION_FACTOR = Math.PI * WHEEL_DIAMETER_METERS / DRIVE_REDUCTION;
    public static final double DRIVE_VELOCITY_FACTOR = DRIVE_POSITION_FACTOR / 60.0;
    public static final double TURN_MOTOR_POSITION_FACTOR = 2.0 * Math.PI / TURN_REDUCTION;
    public static final double TURN_MOTOR_VELOCITY_FACTOR = TURN_MOTOR_POSITION_FACTOR / 60.0;

    // #endregion

    // #region 🛑🚧 SEGURANÇA - LIMITES DOS MOTORES

    // Limites do template REV para o MAXSwerve: 50 A no NEO de tração e 20 A no
    // NEO 550 do giro (o motor pequeno esquenta rápido com corrente alta).
    public static final int DRIVE_CURRENT_LIMIT_AMPS = 50;
    public static final int TURN_CURRENT_LIMIT_AMPS = 20;

    // Compensação de tensão: saída 1,0 = 12 V em qualquer carga da bateria, então os
    // ganhos valem igual com a bateria cheia ou no fim da partida.
    public static final double NOMINAL_VOLTS = 12.0;

    // Velocidade máxima de cada roda: nenhuma passa disto (desaturação da cinemática).
    public static final double MAX_SPEED_MPS = DRIVE_FREE_SPEED_MPS;

    // Giro máximo do robô (template REV): 1 volta por segundo.
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 2.0 * Math.PI;

    // 🛑 Leitura do Spark ou do Pigeon mais velha que isto: considerada perdida.
    public static final double MAX_SIGNAL_AGE_SECONDS = 0.10;

    // 🛑 A REVLib 2026 conversa com o firmware 26.x dos Spark. A 27.x (temporada 2027)
    // não usa os fatores de conversão: m/s e radianos viram RPM e voltas, a tração oscila
    // e o giro não acerta. Firmware fora da 26.x: o módulo não anda e a Driver Station
    // avisa. Em 18/11/2026 os 8 Spark MAX estavam na 25.0.4 (tração "pulando", giro
    // parado) e a equipe atualizou todos para a 26.1.5.
    public static final int SPARK_FIRMWARE_MAJOR = 26;
    public static final String SPARK_FIRMWARE_RECOMMENDED = "26.1.5";

    // 🛑 Nenhum pedido de movimento por este tempo (comando travou): para os módulos.
    public static final double MOTOR_SAFETY_SECONDS = 0.25;

    // 🛑 Mais que isto num ciclo de 20 ms (25 m/s) é impossível: o Spark da tração
    // reiniciou e o encoder voltou a 0. A odometria ignora esse salto.
    public static final double MAX_DRIVE_STEP_METERS = 0.5;

    // #endregion

    // #region 🎯 CONTROLE - TRAÇÃO (VELOCIDADE NO SPARK MAX)

    // kP em fração da saída por m/s de erro; kS e kV em volts (feedForward da REVLib 2026).
    // Template REV: kP = 0,04 e kV = 12 V / velocidade livre (~2,5 V por m/s).
    // 🚧 Para o ajuste fino, medir kS e kV no robô (SysId ou rampa de tensão).
    public static final double DRIVE_KP = 0.04;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KS = 0.0;
    public static final double DRIVE_KV = NOMINAL_VOLTS / DRIVE_FREE_SPEED_MPS;

    // Medição de velocidade do NEO mais rápida que o padrão do Spark (32 ms, média
    // de 8 amostras): menos atraso para o PID de velocidade e para a odometria.
    public static final int DRIVE_ENCODER_MEASUREMENT_PERIOD_MS = 10;
    public static final int DRIVE_ENCODER_AVERAGE_DEPTH = 2;

    // #endregion

    // #region 🎯 CONTROLE - GIRO (POSIÇÃO NO SPARK MAX, ENCODER DO NEO 550)

    // O encoder absoluto está no roboRIO, então o PID roda no Spark (1 kHz) com o encoder
    // do NEO 550, alinhado pelo absoluto ao ligar. kP em fração da saída por radiano de
    // erro do módulo, o mesmo do template REV. O código pede sempre o caminho mais curto.
    public static final double TURN_KP = 1.0;
    public static final double TURN_KI = 0.0;
    public static final double TURN_KD = 0.0;

    // #endregion

    // #region 🛑 GIRO - ALINHAMENTO E CONFERÊNCIA PELO ENCODER ABSOLUTO

    // Com o módulo parado, encoder do NEO 550 longe do absoluto mais que isto: realinha.
    // Desabilitado, só a folga das engrenagens (~0,5°) fica. Habilitado, só um erro de
    // verdade (ex.: Spark do giro reiniciou e voltou a 0).
    public static final double TURN_RESYNC_DISABLED_DEGREES = 2.0;
    public static final double TURN_RESYNC_ENABLED_DEGREES = 10.0;

    // A cada 45° que o módulo gira (conferido parado há 100 ms), o absoluto tem que andar
    // entre 0,7 e 1,4 vezes o que o NEO 550 andou, no mesmo sentido. Errado 2 vezes
    // seguidas: sentido invertido, DIO de outro módulo ou absoluto parado PARAM o módulo;
    // proporção fora (GIRO_NAO_CONFERE) só avisa e o módulo continua.
    // 19/11/2026: era 15° e a proporção fora também parava: a TE parou com razão 1,54
    // (atraso da leitura do Spark pela CAN + folga, alguns graus, pesam muito em 15°).
    public static final double TURN_CHECK_WINDOW_DEGREES = 45.0;
    public static final double TURN_CHECK_MIN_RATIO = 0.7;
    public static final double TURN_CHECK_MAX_RATIO = 1.4;

    // #endregion

    // #region 🎮 PILOTAGEM (TELEOP)

    // Zona morta dos analógicos (o template REV usa 0,05; Xbox gasto passa disso).
    // 19/11/2026: 0,08 → 0,12. Nos logs do Xbox da equipe (09/11/2026), com o analógico
    // solto ele parou em até 0,078 (esquerdo) e 0,094 (direito): com 0,08 o robô podia
    // "andar sozinho" devagar. A saída começa do 0 na borda da zona morta (sem tranco).
    public static final double JOYSTICK_DEADBAND = 0.12;

    // 🚀 Velocidades máximas no teleop e fração usada com LT segurado (modo lento).
    public static final double TELEOP_MAX_SPEED_MPS = MAX_SPEED_MPS;
    public static final double TELEOP_MAX_ANGULAR_SPEED_RAD_PER_SEC = MAX_ANGULAR_SPEED_RAD_PER_SEC;
    public static final double TELEOP_SLOW_SCALE = 0.35;
    // 🐢 BACK (botão ao lado do START) liga/desliga a velocidade reduzida: 30% a menos no
    // andar e no giro (pedido da equipe em 23/11/2026). Vale junto com o LT: com as duas,
    // 0,70 × 0,35 ≈ 25% da velocidade.
    public static final double TELEOP_REDUCED_SCALE = 0.70;

    // 🚧 Limite de aceleração do pedido do piloto: protege as engrenagens de 3" e
    // evita tombar com o pivô alto. Com 8 m/s², 0 a 4,8 m/s em 0,6 s.
    // Aumentar se responder devagar; diminuir se patinar ou balançar ao frear.
    public static final double TELEOP_MAX_ACCELERATION_MPS2 = 8.0;
    public static final double TELEOP_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC2 = 8.0 * Math.PI;

    // #endregion

    // #region 🧪 SIMULAÇÃO (ESTIMATIVAS)

    public static final DCMotor DRIVE_SIM_MOTOR = DCMotor.getNEO(1);
    public static final DCMotor TURN_SIM_MOTOR = DCMotor.getNeo550(1);

    // 🚧 Robô completo com bateria e para-choques: estimado, não pesado.
    public static final double SIM_ROBOT_MASS_KG = 50.0;

    // 🚧 Aderência da banda da roda no carpete (estimada).
    public static final double SIM_WHEEL_COF = 1.1;

    // 🚧 Eixo do giro: inércia do módulo e arrasto da roda no carpete (estimados).
    public static final double SIM_TURN_MOI_KG_M2 = 0.004;
    public static final double SIM_TURN_FRICTION_NM = 0.5;

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region ⚙️ CONSTRUÇÃO

    private SwerveConstants() {
    }

    // #endregion

    // #region 📏 REDUÇÃO DA TRAÇÃO

    /** Voltas do NEO por volta da roda: (45 × engrenagem) / (pinhão × 15), fórmula da REV. */
    public static double driveReduction(int pinionTeeth, int spurTeeth) {
        return (45.0 * spurTeeth) / (pinionTeeth * 15.0);
    }

    // #endregion

    // #endregion
}
