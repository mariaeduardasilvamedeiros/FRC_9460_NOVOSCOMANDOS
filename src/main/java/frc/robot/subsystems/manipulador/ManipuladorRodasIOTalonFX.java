package frc.robot.subsystems.manipulador;

import static frc.robot.subsystems.manipulador.ManipuladorConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;

/** Kraken X60 CAN 43: rodas em velocidade, com rampa e watchdog. */
public final class ManipuladorRodasIOTalonFX extends ManipuladorTalonFX implements ManipuladorRodasIO {
    private final VelocityVoltage velocity = new VelocityVoltage(0);
    private final SlewRateLimiter ramp = new SlewRateLimiter(WHEELS_ACCELERATION_ROTATIONS_PER_SEC2);

    public ManipuladorRodasIOTalonFX() {
        super(WHEELS_MOTOR_ID);
        configure(configuration());
    }

    static TalonFXConfiguration configuration() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = WHEELS_INVERTED
            ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimit = WHEELS_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = WHEELS_STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Slot0.kP = WHEELS_KP;
        config.Slot0.kS = WHEELS_KS;
        config.Slot0.kV = WHEELS_KV;
        return config;
    }

    /** Recupera configuração/falha somente por pedido manual com o robô desabilitado. */
    @Override public void resetWhileDisabled() {
        if (DriverStation.isDisabled()) configure(configuration());
    }

    @Override public void setVelocityRpm(double rpm) {
        if (!Double.isFinite(rpm) || rpm == 0) { stop(); return; }
        double rps = MathUtil.clamp(rpm, -WHEELS_MAX_RPM, WHEELS_MAX_RPM) / 60.0;
        if (!send(velocity.withVelocity(ramp.calculate(rps)))) stop();
    }

    @Override public void stop() {
        stopMotor();
        ramp.reset(0);
    }

    @Override public void updateInputs(ManipuladorRodasIOInputs inputs) {
        readSignals();
        if (!connected || !configured || !controlOK || DriverStation.isDisabled()) stop();
        inputs.connected = connected && configured;
        inputs.controlRequestOK = controlOK;
        inputs.configurationStatus = configurationStatus;
        inputs.positionRotations = motor.getPosition(false).getValueAsDouble();
        inputs.velocityRpm = motor.getVelocity(false).getValueAsDouble() * 60.0;
        inputs.appliedVolts = motor.getMotorVoltage(false).getValueAsDouble();
        inputs.currentAmps = motor.getStatorCurrent(false).getValueAsDouble();
    }
}
