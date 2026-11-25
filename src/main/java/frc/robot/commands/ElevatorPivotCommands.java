package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevatorpivot.ElevatorPivot;

/** Todos os comandos reservam o pivo para evitar controles simultaneos. */
public final class ElevatorPivotCommands {
  // #region ↕️ Movimento — fechar e abrir
  public static Command subir(ElevatorPivot pivot) {
    return Commands.runEnd(pivot::subir, pivot::segurar, pivot).withName("Pivo/Subir");
  }

  public static Command descer(ElevatorPivot pivot) {
    return Commands.runEnd(pivot::descer, pivot::segurar, pivot).withName("Pivo/Descer");
  }
  // #endregion

  // #region 📍 Posições fixas — A, B e X
  /** Vai ate a posicao e segue segurando ate outro comando do pivo assumir. */
  public static Command irPara(ElevatorPivot pivot, double openDeg, String name) {
    return Commands.runEnd(() -> pivot.irPara(openDeg), pivot::segurar, pivot)
        .withName("Pivo/" + name);
  }
  // #endregion

  // #region 🎯 Sustentação da posição
  public static Command segurar(ElevatorPivot pivot) {
    return Commands.runEnd(pivot::segurar, pivot::stop, pivot).withName("Pivo/Segurar");
  }
  // #endregion

  // #region 📏 Zero manual com robô desabilitado
  public static Command zerar(ElevatorPivot pivot) {
    return Commands.runOnce(pivot::zeroAtReference, pivot)
        .ignoringDisable(true).withName("Pivo/Zerar");
  }
  // #endregion

  // #region 🛑 Parada em neutro
  /** Neutro enquanto este comando estiver agendado; ao liberar, o padrao volta a segurar. */
  public static Command parar(ElevatorPivot pivot) {
    return Commands.runEnd(pivot::stop, pivot::stop, pivot).withName("Pivo/Parar");
  }
  // #endregion

  private ElevatorPivotCommands() {}
}
