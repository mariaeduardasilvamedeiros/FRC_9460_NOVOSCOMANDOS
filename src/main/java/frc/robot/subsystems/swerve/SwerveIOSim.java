package frc.robot.subsystems.swerve;

// #region 📚 IMPORTS

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;

// #endregion

/**
 * Um módulo MAXSwerve no computador: NEO + 1/4 da massa do robô na tração e
 * NEO 550 + inércia do módulo + arrasto da roda no giro. Emula os dois Spark MAX
 * como no robô: PID a 1 kHz com os mesmos ganhos, feedforward, compensação de
 * tensão, limite de corrente (smartCurrentLimit) e Brake quando desabilitado.
 * Só números (double), sem objetos por passo.
 *
 * Os ângulos são os do chassi (0 = roda para a frente), como no real. O encoder
 * absoluto simulado é perfeito: sempre com sinal e igual ao ângulo do módulo.
 */
public class SwerveIOSim implements SwerveIO {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    // #region 🧪 FÍSICA

    private static final double LOOP_SECONDS = 0.02;
    private static final double SUBSTEP_SECONDS = 0.001;
    private static final double BATTERY_VOLTS = 12.0;
    private static final double GRAVITY = 9.81;
    // Cada roda empurra 1/4 do robô; a aderência da banda limita a força no chão.
    private static final double WHEEL_MASS_KG = SIM_ROBOT_MASS_KG / 4.0;
    private static final double MAX_WHEEL_FORCE_N = SIM_WHEEL_COF * WHEEL_MASS_KG * GRAVITY;
    // Atrito interno da tração (rolamentos e engrenagens) igual ao que o NEO gasta
    // girando solto (corrente livre): assim a roda livre chega nos ~4,80 m/s da REV.
    private static final double DRIVE_FRICTION_N = DRIVE_SIM_MOTOR.KtNMPerAmp * DRIVE_SIM_MOTOR.freeCurrentAmps
        * DRIVE_REDUCTION / WHEEL_RADIUS_METERS;

    // Tração: velocidade e distância na borda da roda.
    private double driveVelocityMps;
    private double drivePositionMeters;
    private double driveCurrentAmps;
    // Giro: ângulo contínuo do módulo (sem dar a volta em 2π) e velocidade.
    private double turnAngleRad;
    private double turnVelocityRadPerSec;
    private double turnCurrentAmps;

    // #endregion

    // #region 🎯 CONTROLE - ESTADO DOS SPARK MAX EMULADOS

    private boolean active;
    private double targetSpeedMps;
    private double targetAngleRad;
    private double driveIntegral;
    private double driveLastError;
    private double turnIntegral;
    private double turnLastError;
    private double driveVoltsSum;
    private double turnVoltsSum;
    private int voltsSamples;

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region ⚙️ CONSTRUÇÃO

    /** Módulo começando com a roda reta (0 rad). */
    public SwerveIOSim() {
        this(0.0);
    }

    /** Ângulo inicial do módulo (rad): no robô de verdade cada módulo liga torto. */
    public SwerveIOSim(double initialModuleAngleRad) {
        turnAngleRad = initialModuleAngleRad;
    }

    // #endregion

    // #region 🎯 CONTROLE - EMULAÇÃO DOS PID DOS SPARK MAX

    /** Velocidade: kP em fração da saída por m/s (saída 1,0 = 12 V), kS e kV em volts. */
    private double driveControllerVolts() {
        if (!active) return 0.0;
        double error = targetSpeedMps - driveVelocityMps;
        driveIntegral += error;
        double output = DRIVE_KP * error + DRIVE_KI * driveIntegral + DRIVE_KD * (error - driveLastError);
        driveLastError = error;
        double volts = output * NOMINAL_VOLTS + DRIVE_KS * Math.signum(targetSpeedMps) + DRIVE_KV * targetSpeedMps;
        return MathUtil.clamp(volts, -NOMINAL_VOLTS, NOMINAL_VOLTS);
    }

    /** Posição do módulo pelo caminho mais curto (como o SwerveIOSparkMax pede ao Spark). */
    private double turnControllerVolts() {
        if (!active) return 0.0;
        double error = MathUtil.angleModulus(targetAngleRad - turnAngleRad);
        turnIntegral += error;
        double output = TURN_KP * error + TURN_KI * turnIntegral + TURN_KD * (error - turnLastError);
        turnLastError = error;
        return MathUtil.clamp(output, -1.0, 1.0) * NOMINAL_VOLTS;
    }

    // #endregion

    // #region 🛑🚧 SEGURANÇA - LIMITE DE CORRENTE DO SPARK

    /** smartCurrentLimit: V = R·I + ω/Kv, com I preso ao limite. */
    private static double limitCurrent(double volts, double motorRadPerSec, DCMotor motor, double limitAmps) {
        double backEmfVolts = motorRadPerSec / motor.KvRadPerSecPerVolt;
        double limitVolts = limitAmps * motor.rOhms;
        return MathUtil.clamp(MathUtil.clamp(volts, backEmfVolts - limitVolts, backEmfVolts + limitVolts),
            -BATTERY_VOLTS, BATTERY_VOLTS);
    }

    // #endregion

    // #region 🚧 FÍSICA - UM PASSO DE CADA MOTOR

    /** Tração: força do NEO na borda da roda empurra 1/4 da massa do robô. Desabilitado = Brake (0 V). */
    private void stepDrive(boolean enabled, double dtSeconds) {
        double motorRadPerSec = driveVelocityMps / WHEEL_RADIUS_METERS * DRIVE_REDUCTION;
        double volts = enabled
            ? limitCurrent(driveControllerVolts(), motorRadPerSec, DRIVE_SIM_MOTOR, DRIVE_CURRENT_LIMIT_AMPS) : 0.0;
        driveCurrentAmps = DRIVE_SIM_MOTOR.getCurrent(motorRadPerSec, volts);
        double forceN = DRIVE_SIM_MOTOR.getTorque(driveCurrentAmps) * DRIVE_REDUCTION / WHEEL_RADIUS_METERS;
        forceN = MathUtil.clamp(forceN, -MAX_WHEEL_FORCE_N, MAX_WHEEL_FORCE_N);
        driveVelocityMps = withDryFriction(driveVelocityMps, forceN, DRIVE_FRICTION_N, WHEEL_MASS_KG, dtSeconds);
        drivePositionMeters += driveVelocityMps * dtSeconds;
        driveVoltsSum += volts;
    }

    /** Giro: NEO 550 pela redução contra a inércia do módulo e o arrasto da roda no carpete. */
    private void stepTurn(boolean enabled, double dtSeconds) {
        double motorRadPerSec = turnVelocityRadPerSec * TURN_REDUCTION;
        double volts = enabled
            ? limitCurrent(turnControllerVolts(), motorRadPerSec, TURN_SIM_MOTOR, TURN_CURRENT_LIMIT_AMPS) : 0.0;
        turnCurrentAmps = TURN_SIM_MOTOR.getCurrent(motorRadPerSec, volts);
        double torqueNm = TURN_SIM_MOTOR.getTorque(turnCurrentAmps) * TURN_REDUCTION;
        turnVelocityRadPerSec = withDryFriction(turnVelocityRadPerSec, torqueNm, SIM_TURN_FRICTION_NM,
            SIM_TURN_MOI_KG_M2, dtSeconds);
        turnAngleRad += turnVelocityRadPerSec * dtSeconds;
        turnVoltsSum += volts;
    }

    /**
     * Atrito seco: segura parado até o motor vencer e nunca inverte o sentido sozinho.
     * Serve para os dois eixos (força e massa na tração, torque e inércia no giro).
     */
    static double withDryFriction(double velocity, double effort, double friction, double inertia,
            double dtSeconds) {
        if (velocity == 0.0 && Math.abs(effort) <= friction) return 0.0;
        double direction = velocity != 0.0 ? Math.signum(velocity) : Math.signum(effort);
        double next = velocity + (effort - direction * friction) / inertia * dtSeconds;
        boolean reversed = velocity != 0.0 && Math.signum(next) != Math.signum(velocity);
        return reversed && Math.abs(effort) <= friction ? 0.0 : next;
    }

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 🎮 COMANDOS (O SWERVEMODULO CHAMA)

    @Override
    public void setDesiredState(double speedMetersPerSec, double angleRad) {
        if (!Double.isFinite(speedMetersPerSec) || !Double.isFinite(angleRad)) {
            stop();
            return;
        }
        active = true;
        targetSpeedMps = speedMetersPerSec;
        targetAngleRad = angleRad;
    }

    @Override
    public void stop() {
        active = false;
        driveIntegral = 0.0;
        turnIntegral = 0.0;
    }

    // #endregion

    // #region 🧪 CICLO DA SIMULAÇÃO

    /** Avança 20 ms da física (passos de 1 ms). Desabilitado: como o Spark, neutro em Brake. */
    void simulate(boolean enabled) {
        if (!enabled) stop();
        int steps = (int) Math.round(LOOP_SECONDS / SUBSTEP_SECONDS);
        driveVoltsSum = 0.0;
        turnVoltsSum = 0.0;
        for (int i = 0; i < steps; i++) {
            stepDrive(enabled, SUBSTEP_SECONDS);
            stepTurn(enabled, SUBSTEP_SECONDS);
        }
        voltsSamples = steps;
    }

    /** Preenche as leituras como no robô: corrente sempre positiva, ângulo de 0 a 2π. */
    void fillInputs(SwerveIOInputs inputs) {
        inputs.driveConnected = true;
        inputs.turnConnected = true;
        inputs.configurationStatus = "SIMULACAO";
        inputs.driveFirmware = "simulação";
        inputs.turnFirmware = "simulação";
        inputs.activeFaults = "";
        inputs.drivePositionMeters = drivePositionMeters;
        inputs.driveVelocityMetersPerSec = driveVelocityMps;
        inputs.driveAppliedVolts = voltsSamples > 0 ? driveVoltsSum / voltsSamples : 0.0;
        inputs.driveCurrentAmps = Math.abs(driveCurrentAmps);
        inputs.turnPositionRad = MathUtil.inputModulus(turnAngleRad, 0.0, 2.0 * Math.PI);
        inputs.turnVelocityRadPerSec = turnVelocityRadPerSec;
        inputs.turnAppliedVolts = voltsSamples > 0 ? turnVoltsSum / voltsSamples : 0.0;
        inputs.turnCurrentAmps = Math.abs(turnCurrentAmps);
        inputs.absoluteConnected = true;
        inputs.absoluteRawRad = SwerveIOSparkMax.wrapPositive(turnAngleRad);
        inputs.absoluteAngleRad = MathUtil.angleModulus(turnAngleRad);
        inputs.absoluteOffsetRad = 0.0;
        inputs.absoluteFrequencyHz = 976;
        inputs.turnCheckRatio = 1.0;
    }

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        simulate(!DriverStation.isDisabled());
        fillInputs(inputs);
    }

    // #endregion

    // #endregion
}
