package frc.robot.subsystems.elevatorpivot;

/** Constantes de hardware, limites e controle do pivô do elevador. */
public final class ElevatorPivotConstants {
  // #region 🔌 Motores CAN e redução
  // false = os TalonFX 38 e 39 não estão na CAN (ex.: testando só o manipulador):
  // o código não cria os motores, o pivô não se move e o log não enche de erros.
  // Em 18/11/2026 o robô está só com a swerve: false. Voltar para true ao religar o pivô.
  public static final boolean kEnabled = false;
  public static final int kLeaderId = 38;
  public static final int kFollowerId = 39;
  // Duas reducoes de 9:1 e uma de 5:1: total de 405:1.
  public static final double kGearRatio = 9.0 * 9.0 * 5.0;
  // Rotacao livre do motor, usada para calcular o kV abaixo.
  // Pivo com 2x Kraken X44: 7758 RPM trapezoidal (WCP); 7368 RPM com FOC.
  public static final double kMotorFreeSpeedRpm = 7758.0;
  // #endregion

  // #region 🚀 Ajuste de velocidade — abrir e fechar
  // AJUSTE A VELOCIDADE AQUI, em graus do PIVO por segundo.
  // Valores positivos: menor = mais lento; maior = mais rapido.
  // O PID busca esse alvo usando ate kMaxDutyCycle de saida.
  // Maximo fisico sem carga, 2x Kraken X44 com 405:1: ~115 graus/s (~109 com FOC).
  // 100 graus/s = ~87% do maximo: sobra saida para o controle manter a velocidade
  // ao fechar contra a gravidade, com ou sem FOC. Acima de ~115 nao acelera mais.
  // Conferir a velocidade real em VelocidadeRealGrausPorSegundo no painel.
  public static final double kOpenSpeedDegPerSec = 100.0;  // R1/RB: abrir
  public static final double kCloseSpeedDegPerSec = 100.0; // R2/RT: fechar
  // #endregion

  // #region 🐢 Desaceleração perto dos limites
  // Nos ultimos kSlowZoneDeg graus antes do limite do sentido do movimento,
  // o alvo cai em linha reta ate kSlowZoneMinSpeedDegPerSec na chegada.
  // Evita bater nas pontas na velocidade maxima. 0 desliga a rampa.
  public static final double kSlowZoneDeg = 15.0;
  public static final double kSlowZoneMinSpeedDegPerSec = 15.0;
  // #endregion

  // #region 📍 Posições fixas — botões A, B e X
  // Graus de ABERTURA a partir do zero (fechado/de pe). Curso total: ~101.3 graus.
  // Ver a abertura atual em ElevatorPivot/AberturaGraus no painel para ajustar.
  // Valores fora do curso param no limite. A mesma rampa acima e usada perto do alvo.
  public static final double kPositionADeg = 0.0;   // A: fechado (zero)
  public static final double kPositionBDeg = 50.0;  // B: meio
  public static final double kPositionXDeg = 101.0; // X: fim (aberto), 0.3 grau antes do limite
  // A menos disso do alvo, troca o movimento pela sustentacao na posicao.
  public static final double kPositionToleranceDeg = 1.0;
  // #endregion

  // #region 🛡️ Limite de corrente
  public static final double kStatorCurrentLimitAmps = 18.0;
  // #endregion

  // #region 🎯 Ganhos de malha fechada
  // Ganhos agora em fracao de saida: equivalentes nominais a 60 V / 12 V.
  // Continuam provisorios e precisam de ajuste no mecanismo.
  public static final double kPositionKp = 5.0;
  public static final double kVelocityKp = 5.0;
  // kV: saida para cada volta/s do pivo = reducao / rotacao livre do motor em rps.
  // Com 7758 RPM e 405:1: ~3.13. Na velocidade maxima do motor o kV pede 100%.
  // Sem kV, o kP sozinho fica bem abaixo do alvo e o pivo anda devagar.
  public static final double kVelocityKv = kGearRatio / (kMotorFreeSpeedRpm / 60.0);
  // #endregion

  // #region 📡 Atualização e validade das leituras
  public static final double kSignalFrequencyHz = 50.0;
  public static final double kMaxSignalAgeSeconds = 0.1;
  // #endregion

  // #region 📏 Zero físico e limites de posição
  // Zero fisico = fechado/de pe. CSV do principal CAN 38 de 05/11/2026:
  // fechado -8.292969; aberto -122.419922; deslocamento -114.126953.
  // Arredondado para dentro do curso observado. Nao aplicar o offset do CSV
  // ao encoder atual: START define o zero novamente na referencia fisica.
  public static final double kMinRotorPosition = -114.0;
  public static final double kMaxRotorPosition = 0.0;
  // #endregion

  // #region 💪 Ajuda medida para vencer a inércia
  // DutyCycleOut medido no Tuner, em FRACAO, nao volts nem posicao.
  // Levantar/fechar: CAN 38 positivo e CAN 39 negativo (8%).
  // Abrir a partir do zero: CAN 38 negativo e CAN 39 positivo (3%).
  public static final double kUpFeedForwardDutyCycle = 0.08;
  public static final double kDownFeedForwardDutyCycle = 0.03;
  // #endregion

  // #region 🔋 Teto de saída — ajuste da porcentagem máxima
  // 1.0 = 100%; 0.30 = 30%. Limita a saida total, incluindo PID, ajuda e sustentacao.
  // O limite de corrente acima continua valendo com qualquer teto.
  // E um teto: o controle ajusta a saida para buscar a velocidade acima.
  // Nao existe conversao fixa entre porcentagem de saida e graus/s.
  public static final double kMaxDutyCycle = 1;
  // #endregion

  private ElevatorPivotConstants() {}
}
