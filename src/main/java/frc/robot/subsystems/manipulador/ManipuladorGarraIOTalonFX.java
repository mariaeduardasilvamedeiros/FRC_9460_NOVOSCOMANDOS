package frc.robot.subsystems.manipulador;

import static frc.robot.subsystems.manipulador.ManipuladorConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

/** Kraken X60 CAN 42: fecha por motor e abre pela mola em Coast. */
public final class ManipuladorGarraIOTalonFX extends ManipuladorTalonFX implements ManipuladorGarraIO {
    private final MotionMagicVoltage position = new MotionMagicVoltage(0);
    private final VoltageOut hold = new VoltageOut(0);
    private boolean zeroValid;
    private String zeroStatus = "AGUARDANDO_ZERO_AUTOMATICO";
    // A equipe garante a garra aberta ao iniciar. Zerar uma unica vez, desabilitado.
    // Uma perda de referencia durante a operacao nao pode criar um novo zero automatico.
    private boolean startupZeroPending = true;
    private boolean motionMagicActive;

    /** Já configura ao ligar: Coast (a mola abre), limites de corrente e soft limits valem antes do zero. */
    public ManipuladorGarraIOTalonFX() {
        super(CLAW_MOTOR_ID);
        configure(configuration());
    }

    static TalonFXConfiguration configuration() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = CLAW_INVERTED
            ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimit = CLAW_SUPPLY_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = CLAW_STATOR_CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Slot0.kP = CLAW_KP;
        config.Slot0.kD = CLAW_KD;
        config.Slot0.kS = CLAW_KS;
        config.Slot0.kV = CLAW_KV;
        // Como na simulação: kS no sentido do erro, também parada na meta contra a mola.
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        config.MotionMagic.MotionMagicCruiseVelocity = CLAW_CRUISE_ROTATIONS_PER_SEC;
        config.MotionMagic.MotionMagicAcceleration = CLAW_ACCELERATION_ROTATIONS_PER_SEC2;
        // Soft limit 1° além da meta (proteção extra): o Motion Magic para antes dele.
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = CLAW_CLOSED_SOFT_LIMIT_MOTOR_ROTATIONS;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = CLAW_OPEN_LIMIT_MOTOR_ROTATIONS;
        return config;
    }

    /** BACK e opcional na partida; permite recuperar uma falha com a garra aberta. */
    @Override public void zeroHere() {
        if (!DriverStation.isDisabled()) return;
        startupZeroPending = false;
        zeroValid = false;
        boolean ok = configure(configuration());
        readSignals();
        zeroValid = ok && connected && !reset && motor.setPosition(0).isOK();
        zeroStatus = zeroValid ? "ZERO_MANUAL_OK" : "FALHA_AO_ZERAR";
    }

    @Override public void setClosingTargetRotations(double rotations) {
        if (!zeroValid || !Double.isFinite(rotations)) { release(); return; }
        motionMagicActive = send(position.withPosition(MathUtil.clamp(rotations, 0, CLAW_CLOSED_LIMIT_MOTOR_ROTATIONS)));
        if (!motionMagicActive) {
            zeroValid = false;
            zeroStatus = "FALHA_COMANDO_REFACA_ZERO";
        }
    }

    @Override public void holdWithVolts(double volts) {
        if (!zeroValid || !Double.isFinite(volts)) { release(); return; }
        motionMagicActive = false;
        if (!send(hold.withOutput(MathUtil.clamp(volts, 0, 12)))) {
            zeroValid = false;
            zeroStatus = "FALHA_COMANDO_REFACA_ZERO";
        }
    }

    @Override public void release() {
        motionMagicActive = false;
        stopMotor();
    }

    @Override public void updateInputs(ManipuladorGarraIOInputs inputs) {
        readSignals();
        if (zeroValid && (!connected || reset || !controlOK)) {
            zeroValid = false;
            zeroStatus = "FALHA_REFACA_ZERO";
            release();
        }
        if (startupZeroPending) {
            if (!DriverStation.isDisabled()) {
                // Nao adotar outra posicao como aberta depois que a operacao ja comecou.
                startupZeroPending = false;
                zeroStatus = "SEM_ZERO_AUTOMATICO_USE_BACK";
            } else if (connected && configured && controlOK && !reset) {
                zeroValid = motor.setPosition(0).isOK();
                zeroStatus = zeroValid ? "ZERO_AUTOMATICO_OK" : "FALHA_AO_ZERAR";
                startupZeroPending = !zeroValid;
            }
        }
        inputs.connected = connected && configured && controlOK;
        inputs.zeroValid = zeroValid;
        inputs.startupZero = zeroStatus;
        inputs.positionRotations = motor.getPosition(false).getValueAsDouble();
        inputs.velocityRotationsPerSec = motor.getVelocity(false).getValueAsDouble();
        inputs.appliedVolts = motor.getMotorVoltage(false).getValueAsDouble();
        inputs.currentAmps = motor.getStatorCurrent(false).getValueAsDouble();
        // Referência do Motion Magic só enquanto ele está ativo (telemetria).
        inputs.profileReferenceRotations = motionMagicActive
            ? motor.getClosedLoopReference().getValueAsDouble() : inputs.positionRotations;
        inputs.closedLimitActive = inputs.positionRotations >= CLAW_CLOSED_LIMIT_MOTOR_ROTATIONS;
    }
}
