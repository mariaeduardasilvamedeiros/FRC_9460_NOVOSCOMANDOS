// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Configurações gerais do robô e do controle Xbox.
 * As constantes de cada mecanismo ficam na própria pasta em subsystems.
 */
public final class Constants {
  // #region 🎮 Controle Xbox
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    // RT/R2 e considerado pressionado acima de 50% do curso.
    public static final double kRightTriggerThreshold = 0.5;
    // LT/L2 (modo lento do swerve) e considerado pressionado acima de 50% do curso.
    public static final double kLeftTriggerThreshold = 0.5;
  }
  // #endregion
}
