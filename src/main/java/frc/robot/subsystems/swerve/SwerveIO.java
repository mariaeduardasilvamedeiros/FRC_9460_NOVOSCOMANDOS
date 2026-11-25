package frc.robot.subsystems.swerve;

/**
 * Contrato de UM módulo MAXSwerve (Spark MAX da tração + Spark MAX do giro + encoder
 * absoluto no roboRIO): real, simulação ou desligado. O Swerve cria quatro, um por canto.
 *
 * Os ângulos daqui já são do chassi (0 = roda reta para a frente): o offset do encoder
 * absoluto fica dentro do IO. Quem otimiza o estado é o SwerveModulo.
 */
public interface SwerveIO extends AutoCloseable {
    @Override default void close() {}

    /** Motores desligados nas constantes (SWERVE_MOTORS_ENABLED = false): nunca fica pronto. */
    static SwerveIO desligado() {
        return new SwerveIO() {
            @Override public void updateInputs(SwerveIOInputs inputs) {
                inputs.driveConnected = false;
                inputs.turnConnected = false;
                inputs.configurationStatus = "DESLIGADO_NAS_CONSTANTES";
            }
        };
    }

    // #region 📦 PARTE 1 - VARIÁVEIS E TIPOS

    // #region 📡 ENTRADAS

    public static class SwerveIOInputs {
        public boolean driveConnected = false;
        public boolean turnConnected = false;
        public String configurationStatus = "NAO_CONFIGURADO";
        // Diagnóstico: firmware dos dois Spark e falhas ativas (só aviso, não para o módulo).
        public String driveFirmware = "";
        public String turnFirmware = "";
        public String activeFaults = "";
        // Tração: metros e m/s na borda da roda (fatores de conversão no Spark).
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        // Giro: ângulo do módulo no chassi (rad), anti-horário positivo visto de cima.
        // No robô vem do encoder do NEO 550, alinhado pelo absoluto.
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
        // Encoder absoluto (Through Bore no roboRIO): leitura anti-horária sem offset
        // (0 a 2π; com a roda reta, é o offset a calibrar) e ângulo no chassi (−π a π).
        public boolean absoluteConnected = false;
        public double absoluteRawRad = 0.0;
        public double absoluteAngleRad = 0.0;
        // Leitura do absoluto com a roda reta (a de SwerveConstants ou a do zero ao ligar);
        // NaN enquanto não se sabe (ex.: absoluto sem sinal).
        public double absoluteOffsetRad = Double.NaN;
        public int absoluteFrequencyHz = 0;
        // Conferência: quanto o absoluto andou por radiano do NEO 550 na última janela de
        // 15° (~1,0 certo; negativo = sentido oposto). NaN antes da primeira.
        public double turnCheckRatio = Double.NaN;
    }

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES - CONTRATO DO IO

    // #region 🎮 COMANDOS

    /** Velocidade da roda (m/s) e ângulo do módulo (rad, no referencial do chassi). */
    default void setDesiredState(double speedMetersPerSec, double angleRad) {}

    /** Neutro nos dois motores (Brake, como no template REV). */
    default void stop() {}

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 📡 LEITURAS

    default void updateInputs(SwerveIOInputs inputs) {}

    // #endregion

    // #endregion
}
