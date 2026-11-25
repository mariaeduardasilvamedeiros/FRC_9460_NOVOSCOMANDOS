package frc.robot.subsystems.elevatorpivot;

/**
 * Pivô desligado nas constantes (ElevatorPivotConstants.kEnabled = false): não cria
 * os TalonFX 38 e 39, então não há erros de CAN no log. Nunca configura nem se move.
 */
final class ElevatorPivotIODesligado implements ElevatorPivotIO {
  @Override public boolean configure() { return false; }

  @Override public boolean zeroPositions() { return false; }

  // As leituras ficam inválidas (positionsHealthy = false): o pivô nunca fica pronto.
  @Override public void updateInputs(Inputs inputs) {}

  @Override public boolean setVelocity(double pivotRotationsPerSecond, double feedForwardDutyCycle) {
    return false;
  }

  @Override public boolean holdPosition(double pivotRotations) { return false; }

  @Override public void stop() {}
}
