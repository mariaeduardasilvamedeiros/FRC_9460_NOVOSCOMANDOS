package frc.robot.subsystems.elevatorpivot;

/** Fronteira entre a logica do pivo e os controladores fisicos. */
interface ElevatorPivotIO {
  // #region 📊 Leituras dos motores
  class Inputs {
    boolean positionsHealthy;
    boolean resetOccurred;
    double position = Double.NaN;
    double followerPosition = Double.NaN;
    double velocityRps = Double.NaN;
    double currentAmps;
    double followerCurrentAmps;
    double voltage;
    double dutyCycle;
    double followerDutyCycle;
  }
  // #endregion

  // #region ⚙️ Configuração e referência de zero
  boolean configure();
  boolean zeroPositions();
  // #endregion

  // #region 📡 Atualização das leituras
  void updateInputs(Inputs inputs);
  // #endregion

  // #region 🎯 Movimento, sustentação e parada
  boolean setVelocity(double pivotRotationsPerSecond, double feedForwardDutyCycle);
  boolean holdPosition(double pivotRotations);
  void stop();
  // #endregion
}
