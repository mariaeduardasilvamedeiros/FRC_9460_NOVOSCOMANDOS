package frc.robot.subsystems.manipulador;

/** Contrato do motor que gira o manipulador (Spark Flex com NEO Vortex): real, simulação ou replay. */
public interface ManipuladorGiroIO extends AutoCloseable {
    @Override default void close() {}

    /** Motor desligado nas constantes (ROTATION_MOTOR_ENABLED = false): nunca fica pronto. */
    static ManipuladorGiroIO desligado() {
        return new ManipuladorGiroIO() {
            @Override public void updateInputs(ManipuladorGiroIOInputs inputs) {
                inputs.connected = false;
                inputs.zeroValid = false;
                inputs.startupZero = "DESLIGADO_NAS_CONSTANTES";
            }
        };
    }

    // #region 📦 PARTE 1 - VARIÁVEIS E TIPOS

    // #region 📡 ENTRADAS

    public static class ManipuladorGiroIOInputs {
        public boolean connected = false;
        public boolean zeroValid = false;
        public String startupZero = "NAO_AVALIADO";
        // Voltas do rotor; 0 = como no desenho (braços na horizontal).
        public double positionRotations = 0.0;
        public double velocityRotationsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double profileReferenceRotations = 0.0;
        public boolean forwardLimitActive = false;
        public boolean reverseLimitActive = false;
    }

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES - CONTRATO DO IO

    // #region 🎮 COMANDOS

    /** Meta em voltas do rotor; o MAXMotion do Spark Flex faz o perfil. */
    default void setTargetRotations(double motorRotations) {}

    /** Solicita a parada do acionamento pela implementação do IO. */
    default void stop() {}

    /** Desabilitado, com o manipulador como no desenho: a posição atual vira 0. */
    default void zeroHere() {}

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 🎮 COMANDOS

    default void updateInputs(ManipuladorGiroIOInputs inputs) {}

    // #endregion

    // #endregion
}
