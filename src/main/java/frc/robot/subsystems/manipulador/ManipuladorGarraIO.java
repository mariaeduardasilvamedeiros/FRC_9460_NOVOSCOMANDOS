package frc.robot.subsystems.manipulador;

/** Contrato do motor que fecha a garra (1 X60): real, simulação ou replay. A mola abre. */
public interface ManipuladorGarraIO extends AutoCloseable {
    @Override default void close() {}

    /** Motor desligado nas constantes (CLAW_MOTOR_ENABLED = false): nunca fica pronto. */
    static ManipuladorGarraIO desligado() {
        return new ManipuladorGarraIO() {
            @Override public void updateInputs(ManipuladorGarraIOInputs inputs) {
                inputs.connected = false;
                inputs.zeroValid = false;
                inputs.startupZero = "DESLIGADO_NAS_CONSTANTES";
            }
        };
    }

    // #region 📦 PARTE 1 - VARIÁVEIS E TIPOS

    // #region 📡 ENTRADAS

    public static class ManipuladorGarraIOInputs {
        public boolean connected = false;
        public boolean zeroValid = false;
        public String startupZero = "NAO_AVALIADO";
        // Voltas do rotor; 0 = aberta pela mola, positivo = fechando.
        public double positionRotations = 0.0;
        public double velocityRotationsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double profileReferenceRotations = 0.0;
        public boolean closedLimitActive = false;
    }

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES - CONTRATO DO IO

    // #region 🎮 COMANDOS

    /** Fecha pelo Motion Magic do TalonFX até esta meta (voltas do rotor). */
    default void setClosingTargetRotations(double motorRotations) {}

    /** Objeto preso: segura com tensão fixa (a corrente travada é a força de aperto). */
    default void holdWithVolts(double volts) {}

    /** Solta o motor (neutro em Coast): a mola abre a garra. */
    default void release() {}

    /** Desabilitado, com a garra aberta pela mola: a posição atual vira 0. */
    default void zeroHere() {}

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 🎮 COMANDOS

    default void updateInputs(ManipuladorGarraIOInputs inputs) {}

    // #endregion

    // #endregion
}
