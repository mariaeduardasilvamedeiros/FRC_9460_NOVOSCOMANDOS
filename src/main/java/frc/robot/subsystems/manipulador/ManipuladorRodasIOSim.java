package frc.robot.subsystems.manipulador;

// #region 📚 IMPORTS

import static frc.robot.subsystems.manipulador.ManipuladorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;


// #endregion

/**
 * Rodas do manipulador no computador: 1 X60 + inércia das rodas e o mesmo
 * cálculo de tensão que o TalonFX faz no VelocityVoltage. Só números
 * (double), sem objetos por passo.
 */
public class ManipuladorRodasIOSim implements ManipuladorRodasIO {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    // #region 🧪 FÍSICA

    private static final double LOOP_SECONDS = 0.02;

    // Rodas leves: o motor estabiliza em poucos ms. Com passos de 0,5 ms a
    // conta explícita fica bem dentro do estável.
    private static final double SUBSTEP_SECONDS = 0.0005;
    private static final double BATTERY_VOLTS = 12.0;

    // Eixo das rodas (rad/s) e posição do rotor (voltas).
    private double wheelRadPerSec;
    private double rotorRotations;
    private double currentAmps;

    // #endregion

    // #region 🎯 CONTROLE - ESTADO DO TALONFX EMULADO

    private boolean active;
    private double targetRpm;
    private double profileRpm;
    private double appliedVolts;

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region 🛠️ CONVERSÕES

    /** Converte a velocidade das rodas em rad/s para RPM do rotor usando a redução. */
    private static double rotorRpm(double wheelRadPerSec) {
        return wheelRadPerSec * WHEELS_REDUCTION * 60.0 / (2.0 * Math.PI);
    }

    // #endregion

    // #region 🧪 FÍSICA - UM PASSO

    /**
     * Integra a velocidade das rodas usando corrente em amperes, torque, inércia e passo de tempo em
     * segundos.
     */
    private static double step(double wheelRadPerSec, double currentAmps, double dtSeconds) {
        double motorTorqueNm = WHEELS_SIM_MOTOR.getTorque(currentAmps) * WHEELS_REDUCTION;
        return wheelRadPerSec + motorTorqueNm / WHEELS_MOI_KG_M2 * dtSeconds;
    }

    // #endregion

    // #region 🎯 CONTROLE - EMULAÇÃO DO VELOCITYVOLTAGE (PID DO TALONFX)

    /** O mesmo cálculo do Slot0 do TalonFX, em voltas do rotor por segundo. */
    private double controlVolts(double rpm) {
        double targetRps = profileRpm / 60.0;
        double volts = WHEELS_KS * Math.signum(targetRps) + WHEELS_KV * targetRps
            + WHEELS_KP * (targetRps - rpm / 60.0);
        return MathUtil.clamp(volts, -BATTERY_VOLTS, BATTERY_VOLTS);
    }

    // #endregion

    // #region 🛑🚧 SEGURANÇA - LIMITE DE CORRENTE DO ESTATOR

    /** Como no TalonFX: V = R·I + ω/Kv, com I no limite do estator. */
    private static double limitStatorCurrent(double volts, double wheelRadPerSec) {
        double backEmfVolts = wheelRadPerSec * WHEELS_REDUCTION / WHEELS_SIM_MOTOR.KvRadPerSecPerVolt;
        double limitVolts = WHEELS_STATOR_CURRENT_LIMIT_AMPS * WHEELS_SIM_MOTOR.rOhms;
        return MathUtil.clamp(MathUtil.clamp(volts, backEmfVolts - limitVolts, backEmfVolts + limitVolts),
            -BATTERY_VOLTS, BATTERY_VOLTS);
    }

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 🎮 COMANDOS (O SUBSISTEMA CHAMA)

    /** Solicita a rotação do motor em RPM pelo controle de velocidade desta implementação. */
    @Override
    public void setVelocityRpm(double rpm) {
        if (!Double.isFinite(rpm) || rpm == 0.0) {
            stop();
            return;
        }
        active = true;
        targetRpm = MathUtil.clamp(rpm, -WHEELS_MAX_RPM, WHEELS_MAX_RPM);
    }

    /** Encerra o pedido de movimento ativo e coloca esta implementação no seu modo de parada. */
    @Override
    public void stop() {
        active = false;
        targetRpm = 0.0;
        profileRpm = 0.0;
    }

    // #endregion

    // #region 🧪 CICLO DA SIMULAÇÃO

    /** Avança a física do mecanismo e preenche as leituras e os indicadores do IO virtual. */
    @Override
    public void updateInputs(ManipuladorRodasIOInputs inputs) {
        int steps = (int) Math.round(LOOP_SECONDS / SUBSTEP_SECONDS);
        double voltsSum = 0.0;
        for (int i = 0; i < steps; i++) {
            profileRpm = targetRpm == 0.0 ? 0.0 : profileRpm + MathUtil.clamp(targetRpm - profileRpm,
                -WHEELS_ACCELERATION_ROTATIONS_PER_SEC2 * 60.0 * SUBSTEP_SECONDS,
                WHEELS_ACCELERATION_ROTATIONS_PER_SEC2 * 60.0 * SUBSTEP_SECONDS);
            // Segurança primeiro (desabilitado = 0 V), depois o controle limitado
            // pela corrente do estator. Parado: 0 V (Brake curto-circuita o motor).
            appliedVolts = !DriverStation.isEnabled() || !active ? 0.0
                : limitStatorCurrent(controlVolts(rotorRpm(wheelRadPerSec)), wheelRadPerSec);
            currentAmps = WHEELS_SIM_MOTOR.getCurrent(wheelRadPerSec * WHEELS_REDUCTION, appliedVolts);
            wheelRadPerSec = step(wheelRadPerSec, currentAmps, SUBSTEP_SECONDS);
            rotorRotations += wheelRadPerSec * WHEELS_REDUCTION / (2.0 * Math.PI) * SUBSTEP_SECONDS;
            voltsSum += appliedVolts;
        }
        inputs.connected = true;
        inputs.velocityRpm = rotorRpm(wheelRadPerSec);
        inputs.positionRotations = rotorRotations;
        // Média do ciclo: é a tensão que o modelo em paralelo recebe.
        inputs.appliedVolts = voltsSum / steps;
        inputs.currentAmps = Math.abs(currentAmps);
        inputs.controlRequestOK = true;
        inputs.configurationStatus = "SIM";
    }

    // #endregion

    // #endregion
}
