package frc.robot.subsystems.elevatorpivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.DriverStation;

/** Dois TalonFX com inversoes previamente verificadas no Phoenix Tuner. */
final class ElevatorPivotIOTalonFX implements ElevatorPivotIO {
  // #region 🔌 Motores e solicitações de controle
  private final TalonFX leader = new TalonFX(ElevatorPivotConstants.kLeaderId);
  private final TalonFX follower = new TalonFX(ElevatorPivotConstants.kFollowerId);
  private final ElevatorPivot.Calibration calibration;
  private final VelocityDutyCycle velocity = new VelocityDutyCycle(0).withSlot(1);
  private final PositionDutyCycle position = new PositionDutyCycle(0).withSlot(0);
  // Os testes no Tuner usaram sinais opostos nos motores mecanicamente ligados.
  private final Follower follow =
      new Follower(ElevatorPivotConstants.kLeaderId, MotorAlignmentValue.Opposed);
  private final NeutralOut neutral = new NeutralOut();
  // #endregion

  // #region ⚙️ Inicialização e configuração dos TalonFX
  ElevatorPivotIOTalonFX(ElevatorPivot.Calibration calibration) {
    this.calibration = calibration;
  }

  @Override
  public boolean configure() {
    var leaderOutput = new MotorOutputConfigs();
    var followerOutput = new MotorOutputConfigs();
    if (!check(leader, leader.getConfigurator().refresh(leaderOutput))
        || !check(follower, follower.getConfigurator().refresh(followerOutput))) {
      return false;
    }
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = leaderOutput.Inverted;
    config.Feedback.SensorToMechanismRatio = calibration.gearRatio();
    config.Slot0.kP = ElevatorPivotConstants.kPositionKp;
    config.Slot1.kP = ElevatorPivotConstants.kVelocityKp;
    config.Slot1.kV = ElevatorPivotConstants.kVelocityKv;
    config.MotorOutput.PeakForwardDutyCycle = calibration.maxDutyCycle();
    config.MotorOutput.PeakReverseDutyCycle = -calibration.maxDutyCycle();
    config.CurrentLimits.StatorCurrentLimit = ElevatorPivotConstants.kStatorCurrentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = calibration.maxPosition();
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = calibration.minPosition();
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    if (!check(leader, leader.getConfigurator().apply(config))) {
      return false;
    }
    // Preserva o ajuste individual, mas Follower.Opposed determina o sentido
    // durante o seguimento a partir da inversao do principal.
    config.MotorOutput.Inverted = followerOutput.Inverted;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    if (!check(follower, follower.getConfigurator().apply(config))) {
      return false;
    }
    // DutyCycle precisa atualizar para o seguidor acompanhar os controles em porcentagem.
    return check(leader, BaseStatusSignal.setUpdateFrequencyForAll(
        ElevatorPivotConstants.kSignalFrequencyHz,
        leader.getPosition(), follower.getPosition(), leader.getVelocity(), leader.getDutyCycle()));
  }
  // #endregion

  // #region 📏 Referência de zero dos encoders
  @Override
  public boolean zeroPositions() {
    boolean leaderOk = check(leader, leader.setPosition(0));
    boolean followerOk = check(follower, follower.setPosition(0));
    return leaderOk && followerOk;
  }
  // #endregion

  // #region 📡 Leitura e validação dos sinais
  @Override
  public void updateInputs(Inputs inputs) {
    var leaderPosition = leader.getPosition();
    var followerPosition = follower.getPosition();
    var leaderVelocity = leader.getVelocity();
    inputs.position = leaderPosition.getValueAsDouble();
    inputs.followerPosition = followerPosition.getValueAsDouble();
    inputs.velocityRps = healthy(leaderVelocity) ? leaderVelocity.getValueAsDouble() : Double.NaN;
    inputs.positionsHealthy = healthy(leaderPosition) && healthy(followerPosition);
    // Ambas as chamadas precisam consumir o indicador de reset.
    boolean leaderReset = leader.hasResetOccurred();
    boolean followerReset = follower.hasResetOccurred();
    inputs.resetOccurred = leaderReset || followerReset;
    inputs.currentAmps = leader.getStatorCurrent().getValueAsDouble();
    inputs.followerCurrentAmps = follower.getStatorCurrent().getValueAsDouble();
    inputs.voltage = leader.getMotorVoltage().getValueAsDouble();
    inputs.dutyCycle = leader.getDutyCycle().getValueAsDouble();
    inputs.followerDutyCycle = follower.getDutyCycle().getValueAsDouble();
  }

  private boolean healthy(BaseStatusSignal signal) {
    return signal.getStatus().isOK()
        && Double.isFinite(signal.getValueAsDouble())
        && signal.getTimestamp().isValid()
        && signal.getTimestamp().getLatency() <= ElevatorPivotConstants.kMaxSignalAgeSeconds;
  }
  // #endregion

  // #region 🎯 Velocidade e posição em malha fechada
  @Override
  public boolean setVelocity(double pivotRotationsPerSecond, double feedForwardDutyCycle) {
    // Nao acionar o principal se o seguidor rejeitar a solicitacao.
    return check(follower, follower.setControl(follow))
        && check(leader, leader.setControl(velocity.withVelocity(pivotRotationsPerSecond)
            .withFeedForward(feedForwardDutyCycle)));
  }

  @Override
  public boolean holdPosition(double pivotRotations) {
    return check(follower, follower.setControl(follow))
        && check(leader, leader.setControl(position.withPosition(pivotRotations)));
  }
  // #endregion

  // #region 🛑 Saída neutra dos dois motores
  @Override
  public void stop() {
    leader.setControl(neutral);
    follower.setControl(neutral);
  }
  // #endregion

  // #region 🩺 Diagnóstico de comunicação CAN
  private boolean check(TalonFX motor, StatusCode status) {
    if (!status.isOK()) {
      DriverStation.reportError("Pivo CAN " + motor.getDeviceID() + ": " + status, false);
    }
    return status.isOK();
  }
  // #endregion
}
