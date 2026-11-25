package frc.robot.subsystems.manipulador;

// #region 📚 IMPORTS

import static frc.robot.subsystems.manipulador.ManipuladorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;


// #endregion

/**
 * Garra no computador: 1 X60 fechando os dois braços (engrenagens engrenadas)
 * contra a mola, o objeto (se houver) travando os braços, e o mesmo Motion
 * Magic do TalonFX (perfil + kP/kS/kV, soft limit e limite de corrente).
 * Só números (double), sem objetos por passo.
 *
 * Posição = fechamento dos braços (0 = aberta pela mola). Solta, em Coast, o
 * motor não freia: a mola volta a garra para o 0.

 */
public class ManipuladorGarraIOSim implements ManipuladorGarraIO {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    // #region 🧪 FÍSICA

    private static final double LOOP_SECONDS = 0.02;
    private static final double SUBSTEP_SECONDS = 0.0005;
    private static final double BATTERY_VOLTS = 12.0;

    // Batentes físicos: a mola segura no 0; as rodas se encostam no outro lado.
    private static final double WHEELS_TOUCH_RADIANS =
        Math.toRadians(CLAW_OPEN_ARM_DEGREES - WHEELS_TOUCH_ARM_DEGREES);

    // O objeto trava os braços neste fechamento (NaN: sem objeto).
    private final double objectRadians;

    // Fechamento dos braços (rad) e velocidade, no eixo das engrenagens.
    private double closingRad;
    private double velocityRadPerSec;
    private double currentAmps;

    // #endregion

    // #region 🎯 CONTROLE - ESTADO DO TALONFX EMULADO (MOTION MAGIC)

    private boolean motionMagic;
    private boolean holding;
    private double holdVolts;
    private double goalRotations;
    private double referenceRotations;
    private double referenceRotationsPerSec;
    private double appliedVolts;
    private double zeroRotorRotations;
    private boolean closedLimit;

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region ⚙️ CONSTRUÇÃO

    /** Garra com o objeto de ManipuladorConstants.SIM_OBJECT_WIDTH_METERS. */
    public ManipuladorGarraIOSim() {
        this(SIM_OBJECT_WIDTH_METERS);
    }

    /** @param objectWidthMeters largura do objeto entre as rodas; 0 = sem objeto */
    public ManipuladorGarraIOSim(double objectWidthMeters) {
        double armDegrees = objectWidthMeters > 0.0 ? armDegreesForGap(objectWidthMeters - WHEEL_SQUISH_METERS) : Double.NaN;
        objectRadians = Math.toRadians(CLAW_OPEN_ARM_DEGREES - armDegrees);
    }

    // #endregion

    // #region 🛠️ CONVERSÕES

    /** Converte a posição física em voltas do motor, respeitando a referência do encoder usada pelo modelo. */
    private double motorRotations() {
        return closingRad / (2.0 * Math.PI) * CLAW_REDUCTION - zeroRotorRotations;
    }

    /** Ângulo de cada braço que deixa esta folga entre as rodas (borda a borda). */
    static double armDegreesForGap(double gapMeters) {
        return Math.toDegrees(Math.asin(MathUtil.clamp(
            (gapMeters / 2.0 + WHEEL_RADIUS_METERS - PIVOT_HALF_SPACING_METERS) / ARM_LENGTH_METERS, -1.0, 1.0)));
    }

    // #endregion

    // #region 🚧 FÍSICA - UM PASSO E BATENTES

    /** Um passo: motor (fechando) contra a mola (abrindo), na inércia dos braços. */
    private static double step(double velocityRadPerSec, double closingRad, double currentAmps, double dtSeconds) {
        double motorNm = CLAW_SIM_MOTOR.getTorque(currentAmps) * CLAW_REDUCTION;
        double springNm = SPRING_PRELOAD_NM + SPRING_RATE_NM_PER_DEGREE * Math.toDegrees(closingRad);
        return velocityRadPerSec + (motorNm - springNm) / CLAW_MOI_KG_M2 * dtSeconds;
    }

    /** Batentes: a mola encosta no 0; o objeto (ou as rodas uma na outra) para os braços. */
    private void applyStops() {
        double stop = Double.isNaN(objectRadians) ? WHEELS_TOUCH_RADIANS : Math.min(objectRadians, WHEELS_TOUCH_RADIANS);
        if (closingRad < 0.0) {
            closingRad = 0.0;
            velocityRadPerSec = Math.max(0.0, velocityRadPerSec);
        } else if (closingRad > stop) {
            closingRad = stop;
            velocityRadPerSec = Math.min(0.0, velocityRadPerSec);
        }
    }

    // #endregion

    // #region 🎯 CONTROLE - EMULAÇÃO DO MOTION MAGIC (PID DO TALONFX)

    /** Perfil trapezoidal em voltas do rotor: acelera, cruza e freia até a meta. */
    private void advanceProfile(double dtSeconds) {
        double distance = goalRotations - referenceRotations;
        double desired = Math.copySign(Math.min(CLAW_CRUISE_ROTATIONS_PER_SEC,
            Math.sqrt(2.0 * CLAW_ACCELERATION_ROTATIONS_PER_SEC2 * Math.abs(distance))), distance);
        double maxChange = CLAW_ACCELERATION_ROTATIONS_PER_SEC2 * dtSeconds;
        referenceRotationsPerSec += MathUtil.clamp(desired - referenceRotationsPerSec, -maxChange, maxChange);
        referenceRotations += referenceRotationsPerSec * dtSeconds;
        if (Math.abs(goalRotations - referenceRotations) < 1e-4 && Math.abs(referenceRotationsPerSec) <= maxChange) {
            referenceRotations = goalRotations;
            referenceRotationsPerSec = 0.0;
        }
    }

    /** Slot0 com UseClosedLoopSign: kS no sentido do erro. */
    private double controllerVolts() {
        double error = referenceRotations - motorRotations();
        double velocity = velocityRadPerSec / (2.0 * Math.PI) * CLAW_REDUCTION;
        double volts = CLAW_KV * referenceRotationsPerSec + CLAW_KP * error
            + CLAW_KD * (referenceRotationsPerSec - velocity);
        double sign = referenceRotationsPerSec != 0.0 ? Math.signum(referenceRotationsPerSec) : Math.signum(error);
        return MathUtil.clamp(volts + CLAW_KS * sign, -BATTERY_VOLTS, BATTERY_VOLTS);
    }

    // #endregion

    // #region 🛑🚧 SEGURANÇA - SOFT LIMIT E LIMITE DE CORRENTE

    /** Soft limit do TalonFX (1° além da meta): ali não empurra mais (abrir é com a mola). */
    private double applySoftLimit(double volts) {
        double position = motorRotations();
        closedLimit = position >= CLAW_CLOSED_LIMIT_MOTOR_ROTATIONS;
        boolean closedSoftLimit = position >= CLAW_CLOSED_SOFT_LIMIT_MOTOR_ROTATIONS;
        boolean openLimit = position <= CLAW_OPEN_LIMIT_MOTOR_ROTATIONS;
        return (closedSoftLimit && volts > 0.0) || (openLimit && volts < 0.0) ? 0.0 : volts;
    }

    /** Como no TalonFX: V = R·I + ω/Kv, com I no limite do estator (a força de aperto). */
    private static double limitStatorCurrent(double volts, double velocityRadPerSec) {
        double backEmfVolts = velocityRadPerSec * CLAW_REDUCTION / CLAW_SIM_MOTOR.KvRadPerSecPerVolt;
        double limitVolts = CLAW_STATOR_CURRENT_LIMIT_AMPS * CLAW_SIM_MOTOR.rOhms;
        return MathUtil.clamp(MathUtil.clamp(volts, backEmfVolts - limitVolts, backEmfVolts + limitVolts),
            -BATTERY_VOLTS, BATTERY_VOLTS);
    }

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 🎮 COMANDOS (O SUBSISTEMA CHAMA)

    /** Solicita uma posição de mecanismo expressa em voltas do rotor do motor, na referência do seu zero. */
    @Override
    public void setClosingTargetRotations(double motorRotations) {
        if (!Double.isFinite(motorRotations)) {
            release();
            return;
        }
        holding = false;
        if (!motionMagic) {
            // O perfil começa de onde a garra está.
            referenceRotations = motorRotations();
            referenceRotationsPerSec = velocityRadPerSec / (2.0 * Math.PI) * CLAW_REDUCTION;
        }
        motionMagic = true;
        goalRotations = MathUtil.clamp(motorRotations, 0.0, CLAW_CLOSED_LIMIT_MOTOR_ROTATIONS);
    }

    /** Seleciona a retenção da garra por tensão positiva, em volts, limitada à bateria do modelo. */
    @Override
    public void holdWithVolts(double volts) {
        motionMagic = false;
        holding = Double.isFinite(volts);
        holdVolts = holding ? MathUtil.clamp(volts, 0.0, BATTERY_VOLTS) : 0.0;
    }

    /** Desliga o fechamento e a retenção; a dinâmica da mola passa a abrir a garra. */
    @Override
    public void release() {
        motionMagic = false;
        holding = false;
    }

    /** Define a posição atual como referência zero do encoder do mecanismo. */
    @Override
    public void zeroHere() {
        zeroRotorRotations += motorRotations();
        release();
    }

    // #endregion

    // #region 🧪 CICLO DA SIMULAÇÃO

    /** Avança a física do mecanismo e preenche as leituras e os indicadores do IO virtual. */
    @Override
    public void updateInputs(ManipuladorGarraIOInputs inputs) {
        int steps = (int) Math.round(LOOP_SECONDS / SUBSTEP_SECONDS);
        double voltsSum = 0.0;
        for (int i = 0; i < steps; i++) {
            if (motionMagic) {
                advanceProfile(SUBSTEP_SECONDS);
            }
            // Segurança primeiro (desabilitado ou solta = motor solto), depois o controle limitado.
            boolean driven = (motionMagic || holding) && DriverStation.isEnabled();
            double requested = holding ? holdVolts : controllerVolts();
            appliedVolts = driven ? limitStatorCurrent(applySoftLimit(requested), velocityRadPerSec) : 0.0;
            currentAmps = driven ? CLAW_SIM_MOTOR.getCurrent(velocityRadPerSec * CLAW_REDUCTION, appliedVolts) : 0.0;
            velocityRadPerSec = step(velocityRadPerSec, closingRad, currentAmps, SUBSTEP_SECONDS);
            closingRad += velocityRadPerSec * SUBSTEP_SECONDS;
            applyStops();
            voltsSum += appliedVolts;
        }
        inputs.connected = true;
        inputs.zeroValid = true;
        inputs.startupZero = "SIMULACAO";
        inputs.positionRotations = motorRotations();
        inputs.velocityRotationsPerSec = velocityRadPerSec / (2.0 * Math.PI) * CLAW_REDUCTION;
        // Média do ciclo: é a tensão que o modelo em paralelo recebe.
        inputs.appliedVolts = voltsSum / steps;
        inputs.currentAmps = Math.abs(currentAmps);
        inputs.profileReferenceRotations = motionMagic ? referenceRotations : inputs.positionRotations;
        inputs.closedLimitActive = closedLimit;
    }

    // #endregion

    // #endregion
}
