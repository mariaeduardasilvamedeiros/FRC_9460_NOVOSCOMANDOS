package frc.robot.subsystems.manipulador;

// #region 📚 IMPORTS

import static frc.robot.subsystems.manipulador.ManipuladorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;


// #endregion

/**
 * Rotação do manipulador no computador: 1 NEO Vortex + a inércia do
 * manipulador inteiro, e o mesmo MAXMotion do Spark Flex (perfil + kP/kS/kV,
 * soft limits e limite de corrente). Só números (double), sem objetos por passo.
 *
 * Motor direto (1:1) com a carga medida pela equipe: 15% de saída para girar,
 * puxando de volta para o 0 e crescendo com o ângulo (ROTATION_SIM_LOAD_AT_END_NM).
 */
public class ManipuladorGiroIOSim implements ManipuladorGiroIO {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    // #region 🧪 FÍSICA

    private static final double LOOP_SECONDS = 0.02;
    private static final double SUBSTEP_SECONDS = 0.001;
    private static final double BATTERY_VOLTS = 12.0;
    private static final double MIN_HARD_STOP_RADIANS =
        Math.toRadians(ROTATION_MIN_DEGREES - ROTATION_HARD_STOP_MARGIN_DEGREES);
    private static final double MAX_HARD_STOP_RADIANS =
        Math.toRadians(ROTATION_MAX_DEGREES + ROTATION_HARD_STOP_MARGIN_DEGREES);

    // Ângulo e velocidade do manipulador (saída da redução). Liga como no desenho.
    private double angleRad;
    private double velocityRadPerSec;
    private double currentAmps;

    // Zero do encoder: posição do rotor que vale 0 (zeroHere move este ponto).
    private double zeroRotorRotations;

    // #endregion

    // #region 🎯 CONTROLE - ESTADO DO SPARK FLEX EMULADO (MAXMOTION)

    private boolean maxMotion;
    private double goalRotations;
    private double referenceRotations;
    private double referenceRotationsPerSec;
    private double referenceRotationsPerSec2;
    private double appliedVolts;

    // #endregion

    // #region 🛑🚧 SEGURANÇA - ESTADO DOS SOFT LIMITS

    private boolean forwardLimit;
    private boolean reverseLimit;

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region 🛠️ CONVERSÕES

    /** Converte a posição física em voltas do motor, respeitando a referência do encoder usada pelo modelo. */
    private double motorRotations() {
        return angleRad / (2.0 * Math.PI) * ROTATION_REDUCTION - zeroRotorRotations;
    }

    /** Converte a velocidade física do mecanismo em voltas/s do rotor do motor. */
    private double motorRotationsPerSec() {
        return velocityRadPerSec / (2.0 * Math.PI) * ROTATION_REDUCTION;
    }

    // #endregion

    // #region 🚧 FÍSICA - UM PASSO E BATENTE

    /** Um passo da rotação: motor contra a carga medida e a inércia do manipulador. */
    private static double step(double velocityRadPerSec, double angleRad, double currentAmps, double dtSeconds) {
        double torqueNm = ROTATION_SIM_MOTOR.getTorque(currentAmps) * ROTATION_REDUCTION - loadNm(angleRad);
        return velocityRadPerSec + torqueNm / ROTATION_MOI_KG_M2 * dtSeconds;
    }

    /** Carga medida: puxa de volta para o 0, ROTATION_SIM_LOAD_AT_END_NM no fim do curso. */
    static double loadNm(double angleRad) {
        return ROTATION_SIM_LOAD_AT_END_NM * Math.sin(angleRad) / Math.sin(Math.toRadians(ROTATION_END_DEGREES));
    }

    /** 🛑 Batente da fiação, um pouco além do soft limit: para na hora. */
    private void applyHardStop() {
        if (angleRad > MAX_HARD_STOP_RADIANS || angleRad < MIN_HARD_STOP_RADIANS) {
            angleRad = Math.max(MIN_HARD_STOP_RADIANS, Math.min(MAX_HARD_STOP_RADIANS, angleRad));
            velocityRadPerSec = 0.0;
        }
    }

    // #endregion

    // #region 🎯 CONTROLE - EMULAÇÃO DO MAXMOTION (PID DO SPARK FLEX)

    /** Perfil trapezoidal em voltas do rotor: acelera, cruza e freia até a meta. */
    private void advanceProfile(double dtSeconds) {
        double distance = goalRotations - referenceRotations;
        double desired = Math.copySign(Math.min(ROTATION_CRUISE_ROTATIONS_PER_SEC,
            Math.sqrt(2.0 * ROTATION_ACCELERATION_ROTATIONS_PER_SEC2 * Math.abs(distance))), distance);
        double maxChange = ROTATION_ACCELERATION_ROTATIONS_PER_SEC2 * dtSeconds;
        double change = MathUtil.clamp(desired - referenceRotationsPerSec, -maxChange, maxChange);
        referenceRotationsPerSec += change;
        referenceRotationsPerSec2 = change / dtSeconds;
        referenceRotations += referenceRotationsPerSec * dtSeconds;
        if (Math.abs(goalRotations - referenceRotations) < 1e-4 && Math.abs(referenceRotationsPerSec) <= maxChange) {
            referenceRotations = goalRotations;
            referenceRotationsPerSec = 0.0;
            referenceRotationsPerSec2 = 0.0;
        }
    }

    /**
     * Como no Spark: o kP é fração da saída por volta de erro (saída 1,0 =
     * 12 V, pela compensação de tensão); kS, kV e kA são volts e seguem o perfil.
     */
    private double controllerVolts() {
        if (!maxMotion) {
            return 0.0;
        }
        double error = referenceRotations - motorRotations();
        double volts = ROTATION_KP * error * ROTATION_NOMINAL_VOLTS
            + ROTATION_KS * Math.signum(referenceRotationsPerSec)
            + ROTATION_KV * referenceRotationsPerSec
            + ROTATION_KA * referenceRotationsPerSec2;
        return MathUtil.clamp(volts, -ROTATION_MAX_OUTPUT * BATTERY_VOLTS,
            ROTATION_MAX_OUTPUT * BATTERY_VOLTS);
    }

    // #endregion

    // #region 🛑🚧 SEGURANÇA - SOFT LIMITS E LIMITE DE CORRENTE

    /** Soft limits do Spark Flex: bloqueia só o sentido que sai do curso. */
    private double applySoftLimits(double volts) {
        double position = motorRotations();
        forwardLimit = position >= ROTATION_MAX_MOTOR_ROTATIONS;
        reverseLimit = position <= ROTATION_MIN_MOTOR_ROTATIONS;
        return (forwardLimit && volts > 0.0) || (reverseLimit && volts < 0.0) ? 0.0 : volts;
    }

    /** Limite de corrente do Spark (smartCurrentLimit): V = R·I + ω/Kv, com I no limite. */
    private static double limitStatorCurrent(double volts, double velocityRadPerSec) {
        double backEmfVolts = velocityRadPerSec * ROTATION_REDUCTION / ROTATION_SIM_MOTOR.KvRadPerSecPerVolt;
        double limitVolts = ROTATION_CURRENT_LIMIT_AMPS * ROTATION_SIM_MOTOR.rOhms;
        return MathUtil.clamp(MathUtil.clamp(volts, backEmfVolts - limitVolts, backEmfVolts + limitVolts),
            -BATTERY_VOLTS, BATTERY_VOLTS);
    }

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 🎮 COMANDOS (O SUBSISTEMA CHAMA)

    /** Solicita uma posição de mecanismo expressa em voltas do rotor do motor, na referência do seu zero. */
    @Override
    public void setTargetRotations(double motorRotations) {
        if (!Double.isFinite(motorRotations)) {
            stop();
            return;
        }
        if (!maxMotion) {
            // O perfil começa de onde o manipulador está.
            referenceRotations = motorRotations();
            referenceRotationsPerSec = motorRotationsPerSec();
        }
        maxMotion = true;
        goalRotations = MathUtil.clamp(motorRotations, ROTATION_MIN_MOTOR_ROTATIONS, ROTATION_MAX_MOTOR_ROTATIONS);
    }

    /** Encerra o pedido de movimento ativo e coloca esta implementação no seu modo de parada. */
    @Override
    public void stop() {
        maxMotion = false;
    }

    /** Define a posição atual como referência zero do encoder do mecanismo. */
    @Override
    public void zeroHere() {
        zeroRotorRotations += motorRotations();
        stop();
    }

    // #endregion

    // #region 🧪 CICLO DA SIMULAÇÃO

    /** Avança a física do mecanismo e preenche as leituras e os indicadores do IO virtual. */
    @Override
    public void updateInputs(ManipuladorGiroIOInputs inputs) {
        int steps = (int) Math.round(LOOP_SECONDS / SUBSTEP_SECONDS);
        double voltsSum = 0.0;
        for (int i = 0; i < steps; i++) {
            if (maxMotion) {
                advanceProfile(SUBSTEP_SECONDS);
            }
            // Segurança primeiro (desabilitado = 0 V), depois o controle limitado.
            appliedVolts = DriverStation.isDisabled() ? 0.0
                : limitStatorCurrent(applySoftLimits(controllerVolts()), velocityRadPerSec);
            currentAmps = ROTATION_SIM_MOTOR.getCurrent(velocityRadPerSec * ROTATION_REDUCTION, appliedVolts);
            velocityRadPerSec = step(velocityRadPerSec, angleRad, currentAmps, SUBSTEP_SECONDS);
            angleRad += velocityRadPerSec * SUBSTEP_SECONDS;
            applyHardStop();
            voltsSum += appliedVolts;
        }
        inputs.connected = true;
        inputs.zeroValid = true;
        inputs.startupZero = "SIMULACAO";
        inputs.positionRotations = motorRotations();
        inputs.velocityRotationsPerSec = motorRotationsPerSec();
        // Média do ciclo: é a tensão que o modelo em paralelo recebe.
        inputs.appliedVolts = voltsSum / steps;
        inputs.currentAmps = Math.abs(currentAmps);
        inputs.profileReferenceRotations = maxMotion ? referenceRotations : inputs.positionRotations;
        inputs.forwardLimitActive = forwardLimit;
        inputs.reverseLimitActive = reverseLimit;
    }

    // #endregion

    // #endregion
}
