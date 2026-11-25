package frc.robot.subsystems.swerve;

/** Contrato do giroscópio do swerve (Pigeon 2): real, simulação ou desligado. */
public interface SwerveGyroIO extends AutoCloseable {
    @Override default void close() {}

    /** Giroscópio desligado nas constantes (GYRO_ENABLED = false): a frente vem das rodas. */
    static SwerveGyroIO desligado() {
        return new SwerveGyroIO() {
            @Override public void updateInputs(SwerveGyroIOInputs inputs) {
                inputs.connected = false;
                inputs.status = "DESLIGADO_NAS_CONSTANTES";
            }
        };
    }

    // #region 📦 PARTE 1 - VARIÁVEIS E TIPOS

    // #region 📡 ENTRADAS

    public static class SwerveGyroIOInputs {
        public boolean connected = false;
        public String status = "NAO_CONFIGURADO";
        // Contínuo (passa de 360° sem voltar a 0), anti-horário positivo visto de cima.
        public double yawRad = 0.0;
        public double yawVelocityRadPerSec = 0.0;
    }

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES - CONTRATO DO IO

    // #region 🧪 SIMULAÇÃO

    /** Só na simulação: quanto o chassi girou neste ciclo (rad), pela cinemática. O real ignora. */
    default void simulateYawStep(double deltaYawRad) {}

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 📡 LEITURAS

    default void updateInputs(SwerveGyroIOInputs inputs) {}

    // #endregion

    // #endregion
}
