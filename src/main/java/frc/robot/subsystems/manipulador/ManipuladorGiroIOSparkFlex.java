package frc.robot.subsystems.manipulador;

import static frc.robot.subsystems.manipulador.ManipuladorConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Spark Flex CAN 41 com NEO Vortex. Zero manual; nunca presume a posição ao ligar.
 * Só se move com ROTATION_REDUCTION_CONFIRMED = true (redução medida no mecanismo).
 */
public final class ManipuladorGiroIOSparkFlex implements ManipuladorGiroIO {
    // Alertas guardados (sticky) que invalidam o zero: brownout, sensor e reinício.
    private static final int ZERO_WARNINGS_MASK = 0x1 | 0x10 | 0x40;
    // 20° em 20 ms (1000°/s) é impossível para o giro: o encoder voltou a 0 porque
    // o Spark reiniciou. Em graus, para valer com qualquer redução.
    private static final double MAX_JUMP_ROTATIONS = 20.0 / 360.0 * ROTATION_REDUCTION;

    private final SparkFlex motor = new SparkFlex(ROTATION_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    private boolean configured;
    private boolean connected;
    private boolean zeroValid;
    private int warningsAtZero;
    private double lastPositionRotations;
    private int jumpCheckGraceCycles;
    private boolean maxMotionActive;
    private String zeroStatus = ROTATION_REDUCTION_CONFIRMED
        ? "AGUARDANDO_ZERO_MANUAL" : "CONFIRME_ROTATION_REDUCTION";
    /**
     * Já configura ao ligar e grava no Spark: se ele reiniciar no meio do jogo,
     * mantém inversão, Brake, limite de corrente e soft limits.
     */
    public ManipuladorGiroIOSparkFlex() {
        motor.setPeriodicFrameTimeout((int) (MAX_SIGNAL_AGE_SECONDS * 1000));
        stop();
        configured = applyConfiguration(PersistMode.kPersistParameters);
    }

    /**
     * 🛑 A REVLib lança exceção quando o Spark recusa um parâmetro (ex.: firmware
     * do Spark incompatível com a REVLib 2026). Sem este try, o programa do robô
     * inteiro caía ao ligar, junto com o pivô. Assim só o giro fica desligado.
     */
    private boolean applyConfiguration(PersistMode persistMode) {
        try {
            return motor.configure(configuration(), ResetMode.kResetSafeParameters, persistMode)
                == REVLibError.kOk;
        } catch (RuntimeException e) {
            zeroValid = false;
            zeroStatus = "FALHA_CONFIG_SPARK";
            DriverStation.reportError("Giro: Spark Flex " + ROTATION_MOTOR_ID + " recusou a configuração ("
                + e.getMessage() + "). Atualize o firmware do Spark para 2026 no REV Hardware Client 2."
                + " O giro fica desligado.", false);
            return false;
        }
    }

    static SparkFlexConfig configuration() {
        var config = new SparkFlexConfig();
        config.inverted(ROTATION_INVERTED).idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ROTATION_CURRENT_LIMIT_AMPS).voltageCompensation(ROTATION_NOMINAL_VOLTS);
        // Posição em voltas e velocidade em voltas/s: mesmos ganhos da simulação.
        config.encoder.positionConversionFactor(1.0).velocityConversionFactor(1.0 / 60.0);
        config.closedLoop.pid(ROTATION_KP, 0, 0)
            .outputRange(-ROTATION_MAX_OUTPUT, ROTATION_MAX_OUTPUT);
        // kA pesa mais com o motor direto: quase toda a força acelera o manipulador.
        config.closedLoop.feedForward.kS(ROTATION_KS).kV(ROTATION_KV).kA(ROTATION_KA);
        config.closedLoop.maxMotion.cruiseVelocity(ROTATION_CRUISE_ROTATIONS_PER_SEC)
            .maxAcceleration(ROTATION_ACCELERATION_ROTATIONS_PER_SEC2)
            .allowedProfileError(ROTATION_TOLERANCE_DEGREES / 360.0 * ROTATION_REDUCTION);
        config.softLimit.forwardSoftLimit(ROTATION_MAX_MOTOR_ROTATIONS).forwardSoftLimitEnabled(true)
            .reverseSoftLimit(ROTATION_MIN_MOTOR_ROTATIONS).reverseSoftLimitEnabled(true);
        config.signals.primaryEncoderPositionPeriodMs(20).primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20)
            .warningsPeriodMs(20).faultsPeriodMs(20)
            // Status 9 = referência do MAXMotion (telemetria); o padrão é 100 ms.
            .maxMotionSetpointPositionPeriodMs(20);
        return config;
    }

    /** BACK, desabilitado e na referencia fisica: obrigatorio apos ligar ou perder o zero. */
    @Override public void zeroHere() {
        if (!DriverStation.isDisabled()) return;
        stop();
        zeroValid = false;
        configured = applyConfiguration(PersistMode.kNoPersistParameters);
        zeroEncoder("ZERO_MANUAL_OK");
    }

    /** A posição atual vira 0; os alertas de agora viram a referência do zero. */
    private void zeroEncoder(String okStatus) {
        boolean cleared = motor.clearFaults() == REVLibError.kOk;
        // Referência dos alertas no zero: só um alerta NOVO depois daqui invalida o
        // zero (funciona com ou sem o clearFaults limpar os alertas guardados).
        warningsAtZero = motor.getStickyWarnings().rawBits & ZERO_WARNINGS_MASK;
        double position = encoder.getPosition();
        boolean readable = motor.getLastError() == REVLibError.kOk && Double.isFinite(position);
        zeroValid = ROTATION_REDUCTION_CONFIRMED && configured && cleared && readable
            && encoder.setPosition(0) == REVLibError.kOk;
        lastPositionRotations = 0.0;
        jumpCheckGraceCycles = 5;
        zeroStatus = !ROTATION_REDUCTION_CONFIRMED ? "CONFIRME_ROTATION_REDUCTION"
            : !configured ? "FALHA_CONFIG_SPARK" : zeroValid ? okStatus : "FALHA_AO_ZERAR";
    }

    @Override public void setTargetRotations(double rotations) {
        if (DriverStation.isDisabled() || !configured || !connected || !zeroValid
                || !Double.isFinite(rotations)) { stop(); return; }
        var result = motor.getClosedLoopController().setSetpoint(
            MathUtil.clamp(rotations, ROTATION_MIN_MOTOR_ROTATIONS, ROTATION_MAX_MOTOR_ROTATIONS),
            ControlType.kMAXMotionPositionControl);
        maxMotionActive = result == REVLibError.kOk;
        if (result != REVLibError.kOk) {
            zeroValid = false;
            zeroStatus = "FALHA_COMANDO_REFACA_ZERO";
            stop();
        }
    }

    @Override public void stop() {
        maxMotionActive = false;
        motor.stopMotor();
    }

    @Override public void updateInputs(ManipuladorGiroIOInputs inputs) {
        inputs.positionRotations = encoder.getPosition();
        boolean ok = motor.getLastError() == REVLibError.kOk && Double.isFinite(inputs.positionRotations);
        inputs.velocityRotationsPerSec = encoder.getVelocity();
        ok &= motor.getLastError() == REVLibError.kOk && Double.isFinite(inputs.velocityRotationsPerSec);
        double output = motor.getAppliedOutput();
        ok &= motor.getLastError() == REVLibError.kOk && Double.isFinite(output);
        double busVolts = motor.getBusVoltage();
        ok &= motor.getLastError() == REVLibError.kOk && Double.isFinite(busVolts);
        inputs.appliedVolts = output * busVolts;
        inputs.currentAmps = motor.getOutputCurrent();
        ok &= motor.getLastError() == REVLibError.kOk && Double.isFinite(inputs.currentAmps);
        var warnings = motor.getStickyWarnings();
        ok &= motor.getLastError() == REVLibError.kOk;
        var faults = motor.getFaults();
        ok &= motor.getLastError() == REVLibError.kOk && faults.rawBits == 0;
        connected = ok;
        int newWarnings = warnings.rawBits & ZERO_WARNINGS_MASK & ~warningsAtZero;
        // Salto impossível em um ciclo: o encoder voltou a 0 porque o Spark reiniciou.
        // Logo depois do zero a leitura antiga ainda pode chegar: não confere por 5 ciclos.
        boolean jumped = jumpCheckGraceCycles == 0
            && Math.abs(inputs.positionRotations - lastPositionRotations) > MAX_JUMP_ROTATIONS;
        if (jumpCheckGraceCycles > 0) jumpCheckGraceCycles--;
        lastPositionRotations = inputs.positionRotations;
        if (zeroValid && (!ok || newWarnings != 0 || jumped)) {
            zeroValid = false;
            zeroStatus = "FALHA_REFACA_ZERO";
        }
        if (DriverStation.isDisabled() || !zeroValid || !ok) stop();
        inputs.connected = connected && configured;
        inputs.zeroValid = zeroValid;
        inputs.startupZero = zeroStatus;
        // Só lê a referência com o MAXMotion ativo: parado, o Spark não envia o
        // status 9 e a REVLib avisava "timed out while waiting for Period Status 9".
        inputs.profileReferenceRotations = maxMotionActive
            ? motor.getClosedLoopController().getMAXMotionSetpointPosition() : inputs.positionRotations;
        inputs.forwardLimitActive = inputs.positionRotations >= ROTATION_MAX_MOTOR_ROTATIONS;
        inputs.reverseLimitActive = inputs.positionRotations <= ROTATION_MIN_MOTOR_ROTATIONS;
    }

    @Override public void close() {
        stop();
        motor.close();
    }
}
