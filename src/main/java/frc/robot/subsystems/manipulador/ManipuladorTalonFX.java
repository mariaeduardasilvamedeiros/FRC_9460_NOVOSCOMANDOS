package frc.robot.subsystems.manipulador;

import static frc.robot.subsystems.manipulador.ManipuladorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.MotorSafety;

/** Comunicação e watchdog comuns aos dois X60, sempre em unidades do rotor. */
abstract class ManipuladorTalonFX extends MotorSafety implements AutoCloseable {
    protected final TalonFX motor;
    private final BaseStatusSignal[] signals;
    private final NeutralOut neutral = new NeutralOut();
    protected boolean configured;
    protected boolean connected;
    protected boolean controlOK = true;
    protected boolean reset;
    protected String configurationStatus = "NAO_CONFIGURADO";

    ManipuladorTalonFX(int id) {
        motor = new TalonFX(id);
        signals = new BaseStatusSignal[] {motor.getPosition(), motor.getVelocity(),
            motor.getMotorVoltage(), motor.getStatorCurrent()};
        setExpiration(MOTOR_SAFETY_SECONDS);
        setSafetyEnabled(false);
        motor.setControl(neutral);
    }

    protected boolean configure(TalonFXConfiguration config) {
        configured = false;
        stopMotor();
        StatusCode result = motor.getConfigurator().apply(config);
        if (result.isOK()) result = BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals);
        configurationStatus = result.toString();
        configured = result.isOK();
        controlOK = configured;
        motor.hasResetOccurred(); // Consome o reset anterior à configuração.
        return configured;
    }

    protected void readSignals() {
        reset = motor.hasResetOccurred();
        connected = BaseStatusSignal.refreshAll(signals).isOK();
        for (var signal : signals) {
            connected &= signal.getStatus().isOK() && Double.isFinite(signal.getValueAsDouble())
                && signal.getTimestamp().isValid()
                && signal.getTimestamp().getLatency() <= MAX_SIGNAL_AGE_SECONDS;
        }
        if (reset) {
            configured = false;
            configurationStatus = "MOTOR_REINICIOU";
        }
        if (!connected || !configured || !controlOK || DriverStation.isDisabled()) stopMotor();
    }

    protected boolean send(ControlRequest request) {
        if (DriverStation.isDisabled() || !configured || !connected || !controlOK) {
            stopMotor();
            return false;
        }
        controlOK = motor.setControl(request).isOK();
        if (controlOK) {
            feed();
            setSafetyEnabled(true);
        } else {
            stopMotor();
        }
        return controlOK;
    }

    @Override public void stopMotor() {
        setSafetyEnabled(false);
        motor.setControl(neutral);
    }

    @Override public String getDescription() { return "Manipulador TalonFX CAN " + motor.getDeviceID(); }

    @Override public void close() {
        stopMotor();
        motor.close();
    }
}
