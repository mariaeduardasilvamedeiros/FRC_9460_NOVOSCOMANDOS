package frc.robot.subsystems.swerve;

/**
 * Pigeon 2 no computador: o yaw é a soma do quanto o chassi girou em cada ciclo,
 * calculado pela cinemática a partir dos módulos simulados (sem deriva nem ruído).
 */
public class SwerveGyroIOSim implements SwerveGyroIO {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    private static final double LOOP_SECONDS = 0.02;

    private double yawRad;
    private double yawVelocityRadPerSec;

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 🧪 CICLO DA SIMULAÇÃO

    @Override
    public void simulateYawStep(double deltaYawRad) {
        if (!Double.isFinite(deltaYawRad)) return;
        yawRad += deltaYawRad;
        yawVelocityRadPerSec = deltaYawRad / LOOP_SECONDS;
    }

    @Override
    public void updateInputs(SwerveGyroIOInputs inputs) {
        inputs.connected = true;
        inputs.status = "SIMULACAO";
        inputs.yawRad = yawRad;
        inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
    }

    // #endregion

    // #endregion
}
