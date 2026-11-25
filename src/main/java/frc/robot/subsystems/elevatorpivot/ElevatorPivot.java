package frc.robot.subsystems.elevatorpivot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Locale;

/** Pivo do elevador. Zero manual; movimento em Teleop ou Test habilitado. */
public class ElevatorPivot extends SubsystemBase {
  // #region 📏 Calibração e conversão dos limites
  /** Limites em voltas do rotor; controle em voltas do pivo. */
  public record Calibration(
      double gearRatio, double minRotor, double maxRotor,
      double upDutyCycle, double downDutyCycle, double maxDutyCycle) {
    public boolean isValid() {
      return Double.isFinite(gearRatio) && gearRatio > 0
          && Double.isFinite(minRotor) && Double.isFinite(maxRotor)
          && minRotor <= 0 && maxRotor >= 0 && minRotor < maxRotor
          && Double.isFinite(minPosition()) && Double.isFinite(maxPosition())
          && Double.isFinite(ElevatorPivotConstants.kOpenSpeedDegPerSec)
          && ElevatorPivotConstants.kOpenSpeedDegPerSec > 0
          && Double.isFinite(ElevatorPivotConstants.kCloseSpeedDegPerSec)
          && ElevatorPivotConstants.kCloseSpeedDegPerSec > 0
          && Double.isFinite(ElevatorPivotConstants.kVelocityKv)
          && ElevatorPivotConstants.kVelocityKv >= 0
          && Double.isFinite(ElevatorPivotConstants.kSlowZoneDeg)
          && ElevatorPivotConstants.kSlowZoneDeg >= 0
          && Double.isFinite(ElevatorPivotConstants.kSlowZoneMinSpeedDegPerSec)
          && ElevatorPivotConstants.kSlowZoneMinSpeedDegPerSec > 0
          && isValidOpening(ElevatorPivotConstants.kPositionADeg)
          && isValidOpening(ElevatorPivotConstants.kPositionBDeg)
          && isValidOpening(ElevatorPivotConstants.kPositionXDeg)
          && Double.isFinite(ElevatorPivotConstants.kPositionToleranceDeg)
          && ElevatorPivotConstants.kPositionToleranceDeg > 0
          && Double.isFinite(upDutyCycle) && upDutyCycle > 0
          && Double.isFinite(downDutyCycle) && downDutyCycle > 0
          && Double.isFinite(maxDutyCycle)
          && maxDutyCycle >= Math.max(upDutyCycle, downDutyCycle) && maxDutyCycle <= 1;
    }

    public double minPosition() {
      return minRotor / gearRatio;
    }

    public double maxPosition() {
      return maxRotor / gearRatio;
    }

    private static boolean isValidOpening(double openDeg) {
      return Double.isFinite(openDeg) && openDeg >= 0;
    }
  }
  // #endregion

  // #region 🧭 Estado e alvos do pivô
  private enum Mode {
    STOPPED, MOVING, HOLDING
  }

  private final ElevatorPivotIO io;
  private final Calibration calibration;
  private final ElevatorPivotIO.Inputs inputs = new ElevatorPivotIO.Inputs();
  private boolean configured;
  private boolean zeroed;
  private Mode mode = Mode.STOPPED;
  private double holdTarget = Double.NaN;
  private double targetVelocityRps;
  private String status = "Aguardando zero com o pendulo fechado";
  // #endregion

  // #region ⚙️ Inicialização do subsistema
  public ElevatorPivot() {
    this(new Calibration(
        ElevatorPivotConstants.kGearRatio,
        ElevatorPivotConstants.kMinRotorPosition,
        ElevatorPivotConstants.kMaxRotorPosition,
        ElevatorPivotConstants.kUpFeedForwardDutyCycle,
        ElevatorPivotConstants.kDownFeedForwardDutyCycle,
        ElevatorPivotConstants.kMaxDutyCycle));
  }

  private ElevatorPivot(Calibration calibration) {
    this(ElevatorPivotConstants.kEnabled
        ? new ElevatorPivotIOTalonFX(calibration) : new ElevatorPivotIODesligado(), calibration);
  }

  // Permite testar a logica e os comandos sem energizar motores.
  ElevatorPivot(ElevatorPivotIO io, Calibration calibration) {
    this.io = io;
    this.calibration = calibration;
    if (io instanceof ElevatorPivotIODesligado) {
      status = "Pivo desligado: ElevatorPivotConstants.kEnabled = false";
    }
  }
  // #endregion

  // #region 📍 Zero manual na posição fechada
  /**
   * Coloque o pendulo fechado/de pe na marca fisica de zero, com robo
   * desabilitado.
   */
  public boolean zeroAtReference() {
    if (io instanceof ElevatorPivotIODesligado) {
      status = "Pivo desligado: ElevatorPivotConstants.kEnabled = false";
      return false;
    }
    if (!DriverStation.isDisabled()) {
      DriverStation.reportWarning("Pivo: zero apenas com robo desabilitado.", false);
      return false;
    }
    stop();
    zeroed = false;
    if (!calibration.isValid()) {
      status = "Confira limites, velocidades e percentuais em ElevatorPivotConstants";
      DriverStation.reportWarning("Pivo: " + status, false);
      return false;
    }

    // Consome tambem os resets de inicializacao antes de configurar/zerar.
    io.updateInputs(inputs);
    if (inputs.resetOccurred) {
      configured = false;
    }
    if (!configured) {
      configured = io.configure();
    }
    if (!configured || !io.zeroPositions()) {
      invalidate("Falha ao configurar ou zerar os motores");
      return false;
    }
    zeroed = true;
    status = "Zero realizado";
    return true;
  }
  // #endregion

  // #region ↕️ Abrir e fechar em malha fechada
  public void subir() {
    movimentar(1);
  }

  public void descer() {
    movimentar(-1);
  }

  private void movimentar(int direction) {
    if (!isReady()) {
      stop();
      return;
    }
    if ((direction > 0 && inputs.position >= calibration.maxPosition())
        || (direction < 0 && inputs.position <= calibration.minPosition())) {
      segurar();
      return;
    }
    // Distancia ate o limite no sentido do movimento, em graus do pivo.
    double distanceDeg = 360.0 * (direction > 0
        ? calibration.maxPosition() - inputs.position
        : inputs.position - calibration.minPosition());
    andar(direction, distanceDeg,
        direction > 0 ? "Fechando (R2/RT)" : "Abrindo (R1/RB)", "perto do limite");
  }

  /**
   * Velocidade no sentido pedido, desacelerando nos ultimos graus antes do ponto
   * de parada.
   */
  private void andar(int direction, double distanceDeg, String label, String slowLabel) {
    // O TalonFX ja usa voltas do mecanismo (SensorToMechanismRatio = 405).
    // Converter graus/s para voltas/s; nao dividir pela reducao novamente.
    double fullSpeedDegPerSec = direction > 0
        ? ElevatorPivotConstants.kCloseSpeedDegPerSec
        : ElevatorPivotConstants.kOpenSpeedDegPerSec;
    double speedDegPerSec = speedNearLimit(fullSpeedDegPerSec, distanceDeg,
        ElevatorPivotConstants.kSlowZoneDeg, ElevatorPivotConstants.kSlowZoneMinSpeedDegPerSec);
    double velocity = direction * speedDegPerSec / 360.0;
    double feedForward = direction > 0 ? calibration.upDutyCycle() : -calibration.downDutyCycle();
    if (!io.setVelocity(velocity, feedForward)) {
      invalidate("Falha CAN ao movimentar; refaca o zero");
      return;
    }
    mode = Mode.MOVING;
    targetVelocityRps = velocity;
    holdTarget = Double.NaN;
    status = label + (speedDegPerSec < fullSpeedDegPerSec ? ", desacelerando " + slowLabel : "");
  }
  // #endregion

  // #region 📍 Posições fixas — A, B e X
  /**
   * Vai ate {@code openDeg} graus de abertura a partir do zero e segura nessa
   * posicao.
   * Usa a mesma velocidade e a mesma desaceleracao de RB/RT, agora em relacao ao
   * alvo.
   */
  public void irPara(double openDeg) {
    if (!isReady()) {
      stop();
      return;
    }
    double target = clampToTravel(-openDeg / 360.0);
    String arrived = String.format(Locale.ROOT, "Na posicao %.1f graus", -target * 360.0);
    // Ja chegou: continua segurando o mesmo alvo ate outro comando assumir.
    if (mode == Mode.HOLDING && Math.abs(holdTarget - target) < 1e-9) {
      segurarEm(target, arrived);
      return;
    }
    double errorDeg = 360.0 * (target - inputs.position);
    if (Math.abs(errorDeg) <= ElevatorPivotConstants.kPositionToleranceDeg) {
      segurarEm(target, arrived);
      return;
    }
    andar(errorDeg > 0 ? 1 : -1, Math.abs(errorDeg),
        String.format(Locale.ROOT, "Indo para %.1f graus", -target * 360.0), "perto do alvo");
  }

  private double clampToTravel(double position) {
    return Math.max(calibration.minPosition(), Math.min(position, calibration.maxPosition()));
  }
  // #endregion

  // #region 🐢 Desaceleração perto dos limites
  /**
   * Alvo reduzido em linha reta nos ultimos {@code zoneDeg} graus antes do
   * limite:
   * velocidade cheia na entrada da zona e {@code minSpeedDegPerSec} na chegada.
   */
  static double speedNearLimit(
      double speedDegPerSec, double distanceDeg, double zoneDeg, double minSpeedDegPerSec) {
    if (zoneDeg <= 0 || distanceDeg >= zoneDeg) {
      return speedDegPerSec;
    }
    double arrivalSpeed = Math.min(minSpeedDegPerSec, speedDegPerSec);
    double fraction = Math.max(0.0, distanceDeg) / zoneDeg;
    return arrivalSpeed + (speedDegPerSec - arrivalSpeed) * fraction;
  }
  // #endregion

  // #region 🎯 Sustentação da posição
  /**
   * Captura o alvo ao entrar em sustentacao; chamadas seguintes preservam esse
   * alvo.
   */
  public void segurar() {
    if (!isReady()) {
      stop();
      return;
    }
    double target = mode == Mode.HOLDING ? holdTarget : clampToTravel(inputs.position);
    segurarEm(target, "Mantendo posicao");
  }

  private void segurarEm(double target, String newStatus) {
    if (!io.holdPosition(target)) {
      invalidate("Falha CAN ao segurar; refaca o zero");
      return;
    }
    mode = Mode.HOLDING;
    holdTarget = target;
    targetVelocityRps = 0;
    status = newStatus;
  }
  // #endregion

  // #region 🛑 Parada em neutro
  /** Saida neutra com Brake configurado; nao e sustentacao ativa. */
  public void stop() {
    io.stop();
    if (mode != Mode.STOPPED) {
      status = "Saida neutra";
    }
    mode = Mode.STOPPED;
    targetVelocityRps = 0;
    holdTarget = Double.NaN;
  }
  // #endregion

  // #region 🛡️ Condições de movimento e tratamento de falhas
  public boolean isReady() {
    return isControlModeEnabled() && configured && zeroed
        && inputs.positionsHealthy
        && Double.isFinite(inputs.position) && Double.isFinite(inputs.followerPosition);
  }

  /**
   * Regra compartilhada pelos comandos e pelo subsistema. Autonomo permanece
   * bloqueado.
   */
  public boolean isControlModeEnabled() {
    return DriverStation.isTeleopEnabled() || DriverStation.isTestEnabled();
  }

  public String getBlockReason() {
    if (io instanceof ElevatorPivotIODesligado) {
      return "Pivo desligado: ElevatorPivotConstants.kEnabled = false";
    }
    if (!calibration.isValid()) {
      return "Confira limites, velocidades e percentuais em ElevatorPivotConstants";
    }
    if (!configured || !zeroed) {
      return "Feche o pendulo e aperte START com o robo desabilitado para zerar";
    }
    if (!inputs.positionsHealthy
        || !Double.isFinite(inputs.position) || !Double.isFinite(inputs.followerPosition)) {
      return "Leitura CAN dos encoders invalida ou antiga";
    }
    if (!isControlModeEnabled()) {
      return "Habilite Teleop ou Test na Driver Station";
    }
    return "";
  }

  private void invalidate(String reason) {
    stop();
    configured = false;
    zeroed = false;
    status = reason;
    DriverStation.reportError("Pivo: " + reason, false);
  }
  // #endregion

  // #region 🔄 Atualização periódica
  @Override
  public void periodic() {
    // #region 🛡️ Supervisão dos motores e modo do robô
    io.updateInputs(inputs);
    if (configured && inputs.resetOccurred) {
      invalidate("Motor reiniciou; refaca o zero");
    } else if (mode != Mode.STOPPED) {
      if (!inputs.positionsHealthy
          || !Double.isFinite(inputs.position) || !Double.isFinite(inputs.followerPosition)) {
        invalidate("Leitura de posicao invalida ou antiga; refaca o zero");
      } else if (!isControlModeEnabled()) {
        stop();
      }
    }
    // #endregion

    // #region 📊 Monitoramento no SmartDashboard
    SmartDashboard.putBoolean("ElevatorPivot/Configurado", configured);
    SmartDashboard.putBoolean("ElevatorPivot/Zerado", zeroed);
    SmartDashboard.putBoolean("ElevatorPivot/Calibracao", calibration.isValid());
    SmartDashboard.putBoolean("ElevatorPivot/Pronto", isReady());
    SmartDashboard.putBoolean("ElevatorPivot/LeiturasOK", inputs.positionsHealthy);
    SmartDashboard.putString("ElevatorPivot/Estado", mode.toString());
    SmartDashboard.putString("ElevatorPivot/Status", status);
    SmartDashboard.putString("ElevatorPivot/Bloqueio", getBlockReason());
    SmartDashboard.putNumber("ElevatorPivot/Posicao", zeroed ? inputs.position : Double.NaN);
    SmartDashboard.putNumber("ElevatorPivot/AberturaGraus",
        zeroed ? -inputs.position * 360.0 : Double.NaN);
    SmartDashboard.putNumber("ElevatorPivot/PosicaoSeguidor",
        zeroed ? inputs.followerPosition : Double.NaN);
    SmartDashboard.putNumber("ElevatorPivot/Alvo", holdTarget);
    SmartDashboard.putNumber("ElevatorPivot/Corrente", inputs.currentAmps);
    SmartDashboard.putNumber("ElevatorPivot/CorrenteSeguidor", inputs.followerCurrentAmps);
    SmartDashboard.putNumber("ElevatorPivot/Tensao", inputs.voltage);
    SmartDashboard.putNumber("ElevatorPivot/SaidaPorcento", inputs.dutyCycle * 100);
    SmartDashboard.putNumber("ElevatorPivot/SaidaSeguidorPorcento", inputs.followerDutyCycle * 100);
    SmartDashboard.putNumber("ElevatorPivot/VelocidadeAlvoGrausPorSegundo",
        targetVelocityRps * 360);
    SmartDashboard.putNumber("ElevatorPivot/VelocidadeRealGrausPorSegundo",
        inputs.velocityRps * 360);
    // #endregion
  }
  // #endregion
}
