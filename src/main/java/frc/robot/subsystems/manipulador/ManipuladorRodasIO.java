package frc.robot.subsystems.manipulador;

/** Contrato do motor das rodas do manipulador (1 X60): real, simulação ou replay. */
public interface ManipuladorRodasIO extends AutoCloseable {
    @Override default void close() {}

    /** Motor desligado nas constantes (WHEELS_MOTOR_ENABLED = false): nunca fica pronto. */
    static ManipuladorRodasIO desligado() {
        return new ManipuladorRodasIO() {
            @Override public void updateInputs(ManipuladorRodasIOInputs inputs) {
                inputs.connected = false;
                inputs.configurationStatus = "DESLIGADO_NAS_CONSTANTES";
            }
        };
    }

    /** Reaplica a configuração após falha, somente com robô desabilitado. */
    default void resetWhileDisabled() {}

    // #region 📦 PARTE 1 - VARIÁVEIS E TIPOS

    // #region 📡 ENTRADAS

    public static class ManipuladorRodasIOInputs {
        public boolean connected = false;
        // Rotor. Positivo = puxando o objeto para dentro.
        public double velocityRpm = 0.0;
        public double positionRotations = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public boolean controlRequestOK = true;
        public String configurationStatus = "NAO_CONFIGURADO";
    }

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES - CONTRATO DO IO

    // #region 🎮 COMANDOS

    /** RPM do rotor; 0 = para freando (Brake). */
    default void setVelocityRpm(double rpm) {}

    /** Solicita a parada do acionamento pela implementação do IO. */
    default void stop() {}

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 🎮 COMANDOS

    default void updateInputs(ManipuladorRodasIOInputs inputs) {}

    // #endregion

    // #endregion
}
