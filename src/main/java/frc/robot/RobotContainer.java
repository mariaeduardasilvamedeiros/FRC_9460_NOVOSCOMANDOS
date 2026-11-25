package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevatorpivot.ElevatorPivotConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ElevatorPivotCommands;
import frc.robot.commands.manipulador.ManipuladorAbrirCommand;
import frc.robot.commands.manipulador.ManipuladorFecharCommand;
import frc.robot.commands.manipulador.ManipuladorGirarManualCommand;
import frc.robot.commands.manipulador.ManipuladorTesteCommand;
import frc.robot.commands.swerve.SwerveDirigirCommand;
import frc.robot.subsystems.elevatorpivot.ElevatorPivot;
import frc.robot.subsystems.manipulador.Manipulador;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;

public class RobotContainer {
  // #region 🔌 Subsistemas e controle
  private final ElevatorPivot m_elevatorPivot;
  private final CommandXboxController m_driverController;
  private Manipulador m_manipulador;
  private Swerve m_swerve;
  // #endregion

  // #region ⚙️ Inicialização e comando padrão
  public RobotContainer() {
    this(new ElevatorPivot(),
        new CommandXboxController(OperatorConstants.kDriverControllerPort));
    // 🛑 Uma falha ao criar o manipulador (ex.: motor recusando a configuração)
    // não pode derrubar o programa inteiro: o pivô continua funcionando.
    try {
      configureManipulador(new Manipulador());
    } catch (RuntimeException e) {
      m_manipulador = null;
      DriverStation.reportError("Manipulador desligado: falha ao iniciar (" + e + ")",
          e.getStackTrace());
    }
    // 🛑 O mesmo para o swerve: uma falha nele não derruba o pivô nem o manipulador.
    try {
      configureSwerve(new Swerve());
    } catch (RuntimeException e) {
      m_swerve = null;
      DriverStation.reportError("Swerve desligado: falha ao iniciar (" + e + ")",
          e.getStackTrace());
    }
  }

  public RobotContainer(ElevatorPivot pivot, CommandXboxController controller) {
    m_elevatorPivot = pivot;
    m_driverController = controller;
    m_elevatorPivot.setDefaultCommand(ElevatorPivotCommands.segurar(m_elevatorPivot));
    configureBindings();
  }
  // #endregion

  // #region 🎮 Xbox — RB abre, RT fecha, A/B/X posições, Y para, START zera
  private void configureBindings() {
    // ⚠️ 25/11/2026: com o pivô desligado (ElevatorPivotConstants.kEnabled = false) estes
    // botões não fazem nada no pivô, e A, B, X, Y e START são do swerve
    // (configureSwerve). Antes de ligar o pivô de novo, trocar os botões de um dos dois.
    var controlEnabled = new Trigger(m_elevatorPivot::isControlModeEnabled);
    var close = m_driverController.rightTrigger(OperatorConstants.kRightTriggerThreshold);
    var open = m_driverController.rightBumper();
    var stop = m_driverController.y();

    // Xbox: RB (R1) abre; RT (R2) fecha. Ambos juntos mantem posicao.
    // Y tem prioridade e aplica neutro.
    close.and(open.negate()).and(stop.negate()).and(controlEnabled)
        .whileTrue(ElevatorPivotCommands.subir(m_elevatorPivot));
    open.and(close.negate()).and(stop.negate()).and(controlEnabled)
        .whileTrue(ElevatorPivotCommands.descer(m_elevatorPivot));
    stop.and(controlEnabled).whileTrue(ElevatorPivotCommands.parar(m_elevatorPivot));

    // A, B e X: posicoes fixas a partir do zero. Um toque basta: o pivo vai ate
    // a posicao e continua segurando ate RB/RT, Y ou outra posicao assumir.
    bindPosition(m_driverController.a(), ElevatorPivotConstants.kPositionADeg, "PosicaoA",
        stop, controlEnabled);
    bindPosition(m_driverController.b(), ElevatorPivotConstants.kPositionBDeg, "PosicaoB",
        stop, controlEnabled);
    bindPosition(m_driverController.x(), ElevatorPivotConstants.kPositionXDeg, "PosicaoX",
        stop, controlEnabled);

    // START somente desabilitado, com o braco na marca fisica de zero.
    // A verificacao de Disabled fica no subsistema: manter START pressionado
    // durante uma troca de modo nao deve criar um novo evento de zero.
    m_driverController.start().onTrue(ElevatorPivotCommands.zerar(m_elevatorPivot));
  }

  private void bindPosition(
      Trigger button, double openDeg, String name, Trigger stop, Trigger controlEnabled) {
    button.and(stop.negate()).and(controlEnabled)
        .onTrue(ElevatorPivotCommands.irPara(m_elevatorPivot, openDeg, name));
  }
  // #endregion

  // #region 🎮 Manipulador real ou simulado — direcional do Xbox
  private void configureManipulador(Manipulador manipulador) {
    m_manipulador = manipulador;
    var enabled = new Trigger(DriverStation::isTeleopEnabled)
        .and(m_driverController.back().negate()).and(m_driverController.start().negate());
    m_driverController.povLeft().and(enabled).whileTrue(new ManipuladorAbrirCommand(manipulador));
    m_driverController.povRight().and(enabled).whileTrue(new ManipuladorFecharCommand(manipulador));
    // Direcional: posicao com MAXMotion; soltar freia e mantem a posicao.
    m_driverController.povUp().and(enabled).whileTrue(new ManipuladorGirarManualCommand(manipulador, +1));
    m_driverController.povDown().and(enabled).whileTrue(new ManipuladorGirarManualCommand(manipulador, -1));
    // Garra: zero automatico ao iniciar aberta, conforme montagem da equipe.
    // BACK desabilitado: zero do giro; tambem permite refazer o zero da garra aberta.
    m_driverController.back().onTrue(Commands.runOnce(manipulador::zeroHere, manipulador)
        .ignoringDisable(true));
    m_driverController.back().and(new Trigger(DriverStation::isEnabled))
        .whileTrue(Commands.run(manipulador::stopAll, manipulador));
    SmartDashboard.putData("Manipulador/Zerar", Commands.runOnce(manipulador::zeroHere, manipulador)
        .ignoringDisable(true));
    SmartDashboard.putData("Manipulador/TesteCompleto", new ManipuladorTesteCommand(manipulador));
  }
  // #endregion

  // #region 🛞 Swerve — analógicos, A freio, B 80%, X 30%, Y arena/chassi, START+A nova frente
  private void configureSwerve(Swerve swerve) {
    m_swerve = swerve;
    // Analógico esquerdo desloca, direito gira. Sem botão de "lento": a velocidade
    // vem de B e X. O Trigger sempre falso mantém o construtor atual.
    // Os eixos do Xbox são positivos para baixo e para a direita.
    swerve.setDefaultCommand(new SwerveDirigirCommand(swerve,
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        () -> -m_driverController.getRightX(),
        new Trigger(() -> false)));

    // A segurado (sem START): freio, rodas em X, até soltar.
    m_driverController.a().and(m_driverController.start().negate())
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(Commands.run(swerve::setX, swerve).withName("Swerve - freio (X)"));

    // START segurado + toque em A: nova frente do piloto, sem mexer na odometria.
    m_driverController.a().onTrue(Commands.runOnce(() -> {
      if (m_driverController.start().getAsBoolean()) swerve.zeroDriverHeading();
    }).ignoringDisable(true).withName("Swerve - nova frente do piloto"));

    // B: 80% ↔ 50%.  X: 30% ↔ 50%.
    m_driverController.b().onTrue(Commands.runOnce(
        () -> swerve.toggleSpeedScale(SwerveConstants.SPEED_FAST))
        .ignoringDisable(true).withName("Swerve - 80%"));
    m_driverController.x().onTrue(Commands.runOnce(
        () -> swerve.toggleSpeedScale(SwerveConstants.SPEED_SLOW))
        .ignoringDisable(true).withName("Swerve - 30%"));

    // Y: arena (campo, começa assim) ↔ chassi (robô).
    m_driverController.y().onTrue(Commands.runOnce(swerve::toggleFieldCentric)
        .ignoringDisable(true).withName("Swerve - arena/chassi"));

    // Sem botão no controle: define a frente E a pose (zeroHeading), pelo painel.
    SmartDashboard.putData("Swerve/ZerarFrente", Commands.runOnce(swerve::zeroHeading)
        .ignoringDisable(true).withName("Swerve - zerar frente"));

    if (ElevatorPivotConstants.kEnabled) {
      DriverStation.reportWarning("A, B, X, Y e START estão no swerve E no pivô (RobotContainer):"
          + " trocar os botões de um dos dois antes de usar o pivô.", false);
    }
  }

  public void stopSwerve() {
    if (m_swerve != null) m_swerve.stop();
  }
  // #endregion

  // #region 🛑 Parada do pivô
  public void stopElevatorPivot() {
    m_elevatorPivot.stop();
    if (m_manipulador != null) m_manipulador.stopAll();
  }
  // #endregion

  // #region 🤖 Comando autônomo
  public Command getAutonomousCommand() {
    return Commands.none();
  }
  // #endregion
}