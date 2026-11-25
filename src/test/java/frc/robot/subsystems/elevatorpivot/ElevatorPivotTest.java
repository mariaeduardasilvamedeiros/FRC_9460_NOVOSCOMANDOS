package frc.robot.subsystems.elevatorpivot;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorPivotCommands;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

class ElevatorPivotTest {
  private static final ElevatorPivot.Calibration CALIBRATION =
      new ElevatorPivot.Calibration(405, -40.5, 81.0, 0.08, 0.03, 0.08);
  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private FakeIO io;
  private ElevatorPivot pivot;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0));
    scheduler.cancelAll();
    scheduler.unregisterAllSubsystems();
    scheduler.getDefaultButtonLoop().clear();
    scheduler.enable();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    setMode(false, false, false);
    io = new FakeIO();
    pivot = new ElevatorPivot(io, CALIBRATION);
  }

  @AfterEach
  void tearDown() {
    scheduler.cancelAll();
    scheduler.unregisterAllSubsystems();
    scheduler.getDefaultButtonLoop().clear();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  private void setMode(boolean enabled, boolean test, boolean autonomous) {
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.setTest(test);
    DriverStationSim.setAutonomous(autonomous);
    DriverStationSim.notifyNewData();
  }

  private void prepareTest() {
    assertTrue(pivot.zeroAtReference());
    setMode(true, true, false);
    pivot.periodic();
    assertTrue(pivot.isReady());
  }

  static Stream<ElevatorPivot.Calibration> invalidCalibrations() {
    return Stream.of(
        new ElevatorPivot.Calibration(405, Double.NaN, 100, 0.08, 0.03, 0.08),
        new ElevatorPivot.Calibration(405, -100, Double.POSITIVE_INFINITY, 0.08, 0.03, 0.08),
        new ElevatorPivot.Calibration(405, 1, 100, 0.08, 0.03, 0.08),
        new ElevatorPivot.Calibration(405, -100, -1, 0.08, 0.03, 0.08),
        new ElevatorPivot.Calibration(405, 0, 0, 0.08, 0.03, 0.08),
        new ElevatorPivot.Calibration(0, -100, 100, 0.08, 0.03, 0.08),
        new ElevatorPivot.Calibration(Double.NaN, -100, 100, 0.08, 0.03, 0.08),
        new ElevatorPivot.Calibration(405, -100, 100, Double.NaN, 0.03, 0.08),
        new ElevatorPivot.Calibration(405, -100, 100, 0.08, Double.POSITIVE_INFINITY, 0.08),
        new ElevatorPivot.Calibration(405, -100, 100, 0, 0.03, 0.08),
        new ElevatorPivot.Calibration(405, -100, 100, 0.08, 0.03, 0.07),
        new ElevatorPivot.Calibration(405, -100, 100, 0.08, 0.03, 1.01));
  }

  @ParameterizedTest
  @MethodSource("invalidCalibrations")
  void invalidCalibrationNeverConfiguresOrMoves(ElevatorPivot.Calibration calibration) {
    var uncalibrated = new ElevatorPivot(io, calibration);
    assertFalse(uncalibrated.zeroAtReference());
    setMode(true, true, false);
    uncalibrated.periodic();
    uncalibrated.subir();
    assertEquals(0, io.configureCalls);
    assertEquals(0, io.velocityCalls);
  }

  @Test
  void pivotTurnedOffInTheConstantsNeverConfiguresOrMoves() {
    var off = new ElevatorPivot(new ElevatorPivotIODesligado(), CALIBRATION);
    assertFalse(off.zeroAtReference());
    setMode(true, true, false);
    off.periodic();
    off.subir();
    off.irPara(50);
    assertFalse(off.isReady());
    assertTrue(off.getBlockReason().contains("kEnabled"), off.getBlockReason());
  }

  @Test
  void cannotMoveBeforeManualZero() {
    setMode(true, true, false);
    pivot.periodic();
    pivot.subir();
    pivot.descer();
    pivot.segurar();
    assertEquals(0, io.velocityCalls);
    assertEquals(0, io.holdCalls);
  }

  @Test
  void zeroIsAllowedOnlyWhileDisabled() {
    prepareTest();
    assertFalse(pivot.zeroAtReference());
    assertEquals(1, io.zeroCalls);
    assertTrue(pivot.isReady());
  }

  @Test
  void configurationFailurePreventsZeroAndMovement() {
    io.configureOk = false;
    assertFalse(pivot.zeroAtReference());
    assertEquals(0, io.zeroCalls);
    setMode(true, true, false);
    pivot.periodic();
    pivot.subir();
    assertFalse(pivot.isReady());
    assertEquals(0, io.velocityCalls);
  }

  @Test
  void zeroFailurePreventsMovement() {
    io.zeroOk = false;
    assertFalse(pivot.zeroAtReference());
    setMode(true, true, false);
    pivot.periodic();
    pivot.subir();
    assertFalse(pivot.isReady());
    assertEquals(0, io.velocityCalls);
  }

  @Test
  void startupResetIsConsumedBeforeZero() {
    io.reset = true;
    prepareTest();
    assertTrue(pivot.isReady());
    assertEquals(1, io.configureCalls);
  }

  @Test
  void motionConvertsDegreesPerSecondAndUsesSignedFeedForward() {
    prepareTest();
    pivot.subir();
    assertEquals(ElevatorPivotConstants.kCloseSpeedDegPerSec / 360, io.velocity, 1e-12);
    assertEquals(0.08, io.feedForward);
    pivot.descer();
    assertEquals(-ElevatorPivotConstants.kOpenSpeedDegPerSec / 360, io.velocity, 1e-12);
    assertEquals(-0.03, io.feedForward);
  }

  @Test
  void slowZoneRampIsLinearAndOnlyInsideTheZone() {
    // Fora da zona: velocidade cheia.
    assertEquals(90, ElevatorPivot.speedNearLimit(90, 15, 15, 15), 1e-12);
    assertEquals(90, ElevatorPivot.speedNearLimit(90, 40, 15, 15), 1e-12);
    // Dentro da zona: linha reta ate a velocidade de chegada.
    assertEquals(52.5, ElevatorPivot.speedNearLimit(90, 7.5, 15, 15), 1e-12);
    assertEquals(15, ElevatorPivot.speedNearLimit(90, 0, 15, 15), 1e-12);
    assertEquals(15, ElevatorPivot.speedNearLimit(90, -2, 15, 15), 1e-12);
    // Zona 0 desliga a rampa.
    assertEquals(90, ElevatorPivot.speedNearLimit(90, 1, 0, 15), 1e-12);
    // Velocidade cheia menor que a de chegada nunca e aumentada.
    assertEquals(10, ElevatorPivot.speedNearLimit(10, 5, 15, 15), 1e-12);
  }

  @Test
  void motionSlowsDownNearTheLimitInTheDirectionOfTravel() {
    double zone = ElevatorPivotConstants.kSlowZoneDeg;
    double arrival = ElevatorPivotConstants.kSlowZoneMinSpeedDegPerSec;
    double close = ElevatorPivotConstants.kCloseSpeedDegPerSec;
    double open = ElevatorPivotConstants.kOpenSpeedDegPerSec;
    assumeTrue(zone > 0 && arrival < Math.min(close, open));
    prepareTest();

    // Metade da zona antes do limite fechado: alvo no meio da rampa.
    io.position = CALIBRATION.maxPosition() - zone / 2 / 360;
    pivot.periodic();
    pivot.subir();
    assertEquals((arrival + (close - arrival) / 2) / 360, io.velocity, 1e-12);
    assertEquals(0.08, io.feedForward);
    // Na mesma posicao, abrir se afasta do limite: velocidade cheia.
    pivot.descer();
    assertEquals(-open / 360, io.velocity, 1e-12);

    // Um quarto da zona antes do limite aberto.
    io.position = CALIBRATION.minPosition() + zone / 4 / 360;
    pivot.periodic();
    pivot.descer();
    assertEquals(-(arrival + (open - arrival) / 4) / 360, io.velocity, 1e-12);
    assertEquals(-0.03, io.feedForward);
    pivot.subir();
    assertEquals(close / 360, io.velocity, 1e-12);
  }

  @Test
  void fixedPositionMovesWithRampAndHoldsExactTarget() {
    double zone = ElevatorPivotConstants.kSlowZoneDeg;
    double arrival = ElevatorPivotConstants.kSlowZoneMinSpeedDegPerSec;
    double open = ElevatorPivotConstants.kOpenSpeedDegPerSec;
    double tolerance = ElevatorPivotConstants.kPositionToleranceDeg;
    assumeTrue(zone > 0 && arrival < open && tolerance < zone / 2);
    prepareTest();
    double openDeg = 30; // Dentro do curso de teste (36 graus abertos).
    double target = -openDeg / 360;

    pivot.irPara(openDeg); // Longe do alvo: velocidade cheia abrindo.
    assertEquals(-open / 360, io.velocity, 1e-12);
    assertEquals(-0.03, io.feedForward);

    io.position = target + zone / 2 / 360; // Metade da zona antes do alvo.
    pivot.periodic();
    pivot.irPara(openDeg);
    assertEquals(-(arrival + (open - arrival) / 2) / 360, io.velocity, 1e-12);

    int moves = io.velocityCalls;
    io.position = target + tolerance / 2 / 360; // Dentro da tolerancia: segura o alvo exato.
    pivot.periodic();
    pivot.irPara(openDeg);
    assertEquals(moves, io.velocityCalls);
    assertEquals(target, io.target, 1e-12);

    io.position = target + 3 * tolerance / 360; // Mexeu depois de chegar: continua segurando.
    pivot.periodic();
    pivot.irPara(openDeg);
    assertEquals(moves, io.velocityCalls);
    assertEquals(target, io.target, 1e-12);
  }

  @Test
  void fixedPositionClosesTowardZeroAndClampsOutsideTravel() {
    prepareTest();
    io.position = -0.05;
    pivot.periodic();
    pivot.irPara(0);
    assertTrue(io.velocity > 0);
    assertEquals(0.08, io.feedForward);
    pivot.irPara(200); // Fora do curso de teste: alvo limitado ao limite aberto.
    assertTrue(io.velocity < 0);
    io.position = CALIBRATION.minPosition();
    pivot.periodic();
    pivot.irPara(200);
    assertEquals(CALIBRATION.minPosition(), io.target, 1e-12);
  }

  @Test
  void positionButtonsGoToFixedPositionsAndStopHasPriority() {
    var measured = new ElevatorPivot.Calibration(
        ElevatorPivotConstants.kGearRatio,
        ElevatorPivotConstants.kMinRotorPosition,
        ElevatorPivotConstants.kMaxRotorPosition,
        ElevatorPivotConstants.kUpFeedForwardDutyCycle,
        ElevatorPivotConstants.kDownFeedForwardDutyCycle,
        ElevatorPivotConstants.kMaxDutyCycle);
    var measuredPivot = new ElevatorPivot(io, measured);
    assertTrue(measuredPivot.zeroAtReference());
    setMode(true, false, false); // Teleop
    measuredPivot.periodic();
    var controller = new CommandXboxController(0);
    var controllerSim = new XboxControllerSim(controller.getHID());
    controllerSim.notifyNewData();
    new RobotContainer(measuredPivot, controller);

    controllerSim.setBButton(true); // B: meio, saindo do fechado.
    controllerSim.notifyNewData();
    scheduler.run();
    assertTrue(io.velocity < 0);
    controllerSim.setBButton(false); // Um toque basta: continua indo apos soltar.
    controllerSim.notifyNewData();
    int moves = io.velocityCalls;
    scheduler.run();
    assertEquals(moves + 1, io.velocityCalls);

    io.position = -ElevatorPivotConstants.kPositionBDeg / 360; // Chegou no meio.
    scheduler.run();
    assertEquals(-ElevatorPivotConstants.kPositionBDeg / 360, io.target, 1e-12);

    controllerSim.setXButton(true); // X: fim.
    controllerSim.notifyNewData();
    scheduler.run();
    assertTrue(io.velocity < 0);
    controllerSim.setXButton(false);
    controllerSim.notifyNewData();

    controllerSim.setYButton(true); // Y para tudo, com prioridade.
    controllerSim.notifyNewData();
    scheduler.run();
    moves = io.velocityCalls;
    int holds = io.holdCalls;
    scheduler.run();
    assertEquals(moves, io.velocityCalls);
    assertEquals(holds, io.holdCalls);

    controllerSim.setAButton(true); // A com Y pressionado nao move.
    controllerSim.notifyNewData();
    scheduler.run();
    assertEquals(moves, io.velocityCalls);
    controllerSim.setAButton(false);
    controllerSim.setYButton(false);
    controllerSim.notifyNewData();
    scheduler.run();

    controllerSim.setAButton(true); // A: volta para o zero, fechando.
    controllerSim.notifyNewData();
    scheduler.run();
    assertTrue(io.velocity > 0);
    assertEquals(0.08, io.feedForward);
  }

  @Test
  void holdKeepsOriginalTargetWhenArmDrifts() {
    prepareTest();
    io.position = 0.15;
    pivot.periodic();
    pivot.segurar();
    io.position = 0.12;
    pivot.periodic();
    pivot.segurar();
    assertEquals(0.15, io.target, 1e-12);
    pivot.subir();
    io.position = 0.17;
    pivot.periodic();
    pivot.segurar();
    assertEquals(0.17, io.target, 1e-12);
  }

  @Test
  void upperLimitHoldsClampedTargetAndAllowsRetreat() {
    prepareTest();
    io.position = 0.21;
    pivot.periodic();
    pivot.subir();
    assertEquals(0, io.velocityCalls);
    assertEquals(0.2, io.target, 1e-12);
    pivot.descer();
    assertEquals(1, io.velocityCalls);
    assertTrue(io.velocity < 0);
  }

  @Test
  void lowerLimitHoldsClampedTargetAndAllowsRetreat() {
    prepareTest();
    io.position = -0.11;
    pivot.periodic();
    pivot.descer();
    assertEquals(0, io.velocityCalls);
    assertEquals(-0.1, io.target, 1e-12);
    pivot.subir();
    assertEquals(1, io.velocityCalls);
    assertTrue(io.velocity > 0);
  }

  @Test
  void measuredConfigurationUsesClosedZeroAndAppliesGearRatioOnce() {
    var measured = new ElevatorPivot.Calibration(
        ElevatorPivotConstants.kGearRatio,
        ElevatorPivotConstants.kMinRotorPosition,
        ElevatorPivotConstants.kMaxRotorPosition,
        ElevatorPivotConstants.kUpFeedForwardDutyCycle,
        ElevatorPivotConstants.kDownFeedForwardDutyCycle,
        ElevatorPivotConstants.kMaxDutyCycle);
    assertTrue(measured.isValid());
    assertEquals(0.0, measured.maxPosition());
    assertEquals(-114.0 / 405.0, measured.minPosition(), 1e-12);
    assertEquals(-101.333333333, measured.minPosition() * 360, 1e-8);
    assertEquals(1.0, measured.maxDutyCycle());
    // kV pede 100% de saida exatamente na rotacao livre do motor, em voltas/s do pivo.
    double freePivotRps = ElevatorPivotConstants.kMotorFreeSpeedRpm / 60 / measured.gearRatio();
    assertEquals(1.0, ElevatorPivotConstants.kVelocityKv * freePivotRps, 1e-12);
    // Posicoes A, B e X dentro do curso medido, na ordem fechado -> meio -> aberto.
    double travelDeg = -measured.minPosition() * 360;
    assertEquals(0.0, ElevatorPivotConstants.kPositionADeg);
    assertTrue(ElevatorPivotConstants.kPositionBDeg > ElevatorPivotConstants.kPositionADeg);
    assertTrue(ElevatorPivotConstants.kPositionXDeg > ElevatorPivotConstants.kPositionBDeg);
    assertTrue(ElevatorPivotConstants.kPositionXDeg <= travelDeg);
    var measuredPivot = new ElevatorPivot(io, measured);
    assertTrue(measuredPivot.zeroAtReference());
    setMode(true, true, false);
    measuredPivot.periodic();
    measuredPivot.subir(); // Fechado: nao levantar alem do zero.
    assertEquals(0, io.velocityCalls);
    measuredPivot.descer(); // Abrir a partir do fechado.
    assertTrue(io.velocity < 0);
    assertEquals(-0.03, io.feedForward);
    io.position = measured.minPosition();
    measuredPivot.periodic();
    int moves = io.velocityCalls;
    measuredPivot.descer(); // Aberto: nao abrir alem do limite.
    assertEquals(moves, io.velocityCalls);
    measuredPivot.subir(); // Retornar do aberto.
    assertTrue(io.velocity > 0);
    assertEquals(0.08, io.feedForward);
  }

  @Test
  void autonomousAndDisabledStopActiveHoldAndBlockMovement() {
    prepareTest();
    pivot.segurar();
    int holds = io.holdCalls;
    int stops = io.stopCalls;
    setMode(true, false, true); // Autonomous
    pivot.periodic();
    assertTrue(io.stopCalls > stops);
    pivot.subir();
    pivot.segurar();
    pivot.descer();
    setMode(false, true, false);
    pivot.periodic();
    pivot.subir();
    assertEquals(0, io.velocityCalls);
    assertEquals(holds, io.holdCalls);
  }

  @Test
  void teleopWorksAfterZeroAndStillRequiresValidFeedback() {
    setMode(true, false, false);
    pivot.periodic();
    pivot.descer();
    assertEquals(0, io.velocityCalls);
    assertTrue(pivot.getBlockReason().contains("START"));
    setMode(false, false, false);
    assertTrue(pivot.zeroAtReference());
    setMode(true, false, false);
    pivot.periodic();
    assertTrue(pivot.isReady());
    assertEquals("", pivot.getBlockReason());
    pivot.descer();
    assertEquals(1, io.velocityCalls);
    assertTrue(io.velocity < 0);
    io.healthy = false;
    pivot.periodic();
    assertFalse(pivot.isReady());
    pivot.descer();
    assertEquals(1, io.velocityCalls);
  }

  @Test
  void sensorFailureDuringHoldRequiresAnotherDisabledZero() {
    prepareTest();
    pivot.segurar();
    io.healthy = false;
    pivot.periodic();
    io.healthy = true;
    pivot.periodic();
    pivot.subir();
    assertFalse(pivot.isReady());
    assertEquals(0, io.velocityCalls);
    setMode(false, false, false);
    prepareTest();
    pivot.subir();
    assertEquals(1, io.velocityCalls);
    assertEquals(2, io.configureCalls);
  }

  @Test
  void nonFiniteFollowerReadingStopsBothMotors() {
    prepareTest();
    pivot.subir();
    int stops = io.stopCalls;
    io.followerPosition = Double.NaN;
    pivot.periodic();
    assertTrue(io.stopCalls > stops);
    assertFalse(pivot.isReady());
  }

  @Test
  void motorResetInvalidatesZeroEvenWhileIdle() {
    prepareTest();
    io.reset = true;
    pivot.periodic();
    assertFalse(pivot.isReady());
    pivot.subir();
    assertEquals(0, io.velocityCalls);
  }

  @Test
  void rejectedMotorRequestStopsAndLatchesFault() {
    prepareTest();
    io.controlOk = false;
    pivot.subir();
    assertFalse(pivot.isReady());
    io.controlOk = true;
    pivot.periodic();
    pivot.subir();
    assertEquals(1, io.velocityCalls);
  }

  @Test
  void commandCancellationCapturesTargetAndDefaultPreservesIt() {
    prepareTest();
    pivot.setDefaultCommand(ElevatorPivotCommands.segurar(pivot));
    Command up = ElevatorPivotCommands.subir(pivot);
    scheduler.schedule(up);
    scheduler.run();
    assertTrue(up.getRequirements().contains(pivot));
    assertEquals(1, io.velocityCalls);
    io.position = 0.1;
    scheduler.run();
    up.cancel();
    assertEquals(0.1, io.target, 1e-12);
    scheduler.run(); // Agenda o comando padrao.
    io.position = 0.08;
    scheduler.run();
    assertEquals(0.1, io.target, 1e-12);
  }

  @Test
  void stopCommandTakesRequirementAndAppliesNeutral() {
    prepareTest();
    Command down = ElevatorPivotCommands.descer(pivot);
    scheduler.schedule(down);
    scheduler.run();
    Command stop = ElevatorPivotCommands.parar(pivot);
    scheduler.schedule(stop);
    scheduler.run();
    assertFalse(down.isScheduled());
    assertTrue(stop.isScheduled());
    int moves = io.velocityCalls;
    int holds = io.holdCalls;
    scheduler.run();
    assertEquals(moves, io.velocityCalls);
    assertEquals(holds, io.holdCalls);
  }

  @Test
  void zeroCommandRunsWhileDisabledButRejectsEnabledZero() {
    scheduler.schedule(ElevatorPivotCommands.zerar(pivot));
    scheduler.run();
    assertEquals(1, io.zeroCalls);
    setMode(true, true, false);
    scheduler.schedule(ElevatorPivotCommands.zerar(pivot));
    scheduler.run();
    assertEquals(1, io.zeroCalls);
  }

  @ParameterizedTest
  @ValueSource(booleans = {false, true})
  void controllerBindingsHandleConflictingButtonsAndStopPriority(boolean testMode) {
    prepareTest();
    setMode(true, testMode, false);
    var controller = new CommandXboxController(0);
    var controllerSim = new XboxControllerSim(controller.getHID());
    controllerSim.notifyNewData();
    new RobotContainer(pivot, controller);
    controllerSim.setRightTriggerAxis(1.0); // R2/RT fecha.
    controllerSim.notifyNewData();
    scheduler.run();
    assertTrue(io.velocity > 0);
    int moves = io.velocityCalls;

    controllerSim.setRightBumperButton(true); // R1 + R2 deve segurar.
    controllerSim.notifyNewData();
    scheduler.run();
    scheduler.run();
    assertEquals(moves, io.velocityCalls);
    assertTrue(io.holdCalls > 0);

    controllerSim.setRightTriggerAxis(0); // R1 continua pressionado: abrir.
    controllerSim.notifyNewData();
    scheduler.run();
    assertTrue(io.velocity < 0);

    controllerSim.setYButton(true); // Y prevalece sobre R1.
    controllerSim.notifyNewData();
    scheduler.run();
    moves = io.velocityCalls;
    int holds = io.holdCalls;
    scheduler.run();
    assertEquals(moves, io.velocityCalls);
    assertEquals(holds, io.holdCalls);

    controllerSim.setYButton(false); // Ao liberar Y com R1 pressionado, retoma abertura.
    controllerSim.notifyNewData();
    scheduler.run();
    assertEquals(moves + 1, io.velocityCalls);
    assertTrue(io.velocity < 0);
  }

  @Test
  void rightTriggerRequiresHalfTravelAndReleaseHoldsPositionInTeleop() {
    prepareTest();
    setMode(true, false, false);
    var controller = new CommandXboxController(0);
    var controllerSim = new XboxControllerSim(controller.getHID());
    controllerSim.notifyNewData();
    new RobotContainer(pivot, controller);
    controllerSim.setLeftBumperButton(true); // Botoes sem funcao nao movem o pivo.
    controllerSim.setBackButton(true);
    controllerSim.setRightTriggerAxis(0.1);
    controllerSim.notifyNewData();
    scheduler.run();
    scheduler.run();
    assertEquals(0, io.velocityCalls);

    controllerSim.setRightTriggerAxis(0.6);
    controllerSim.notifyNewData();
    scheduler.run();
    assertEquals(1, io.velocityCalls);
    assertTrue(io.velocity > 0);
    io.position = 0.1;
    controllerSim.setRightTriggerAxis(0);
    controllerSim.notifyNewData();
    scheduler.run();
    scheduler.run();
    assertEquals(1, io.velocityCalls);
    assertEquals(0.1, io.target, 1e-12);
  }

  @Test
  void holdingStartAcrossDisableDoesNotZeroUnexpectedly() {
    prepareTest();
    var controller = new CommandXboxController(0);
    var controllerSim = new XboxControllerSim(controller.getHID());
    controllerSim.notifyNewData();
    new RobotContainer(pivot, controller);
    controllerSim.setStartButton(true);
    controllerSim.notifyNewData();
    scheduler.run(); // Zero rejeitado porque esta habilitado.
    assertEquals(1, io.zeroCalls);
    setMode(false, false, false);
    scheduler.run(); // Apenas desabilitar nao pode criar um novo zero.
    assertEquals(1, io.zeroCalls);
    controllerSim.setStartButton(false);
    controllerSim.notifyNewData();
    scheduler.run();
    controllerSim.setStartButton(true);
    controllerSim.notifyNewData();
    scheduler.run();
    assertEquals(2, io.zeroCalls);
  }

  private static class FakeIO implements ElevatorPivotIO {
    boolean configureOk = true;
    boolean zeroOk = true;
    boolean controlOk = true;
    boolean healthy = true;
    boolean reset;
    double position;
    double followerPosition;
    double velocity;
    double feedForward;
    double target;
    int configureCalls;
    int zeroCalls;
    int velocityCalls;
    int holdCalls;
    int stopCalls;

    public boolean configure() { configureCalls++; return configureOk; }
    public boolean zeroPositions() {
      zeroCalls++;
      position = 0;
      followerPosition = 0;
      return zeroOk;
    }
    public void updateInputs(Inputs inputs) {
      inputs.position = position;
      inputs.followerPosition = followerPosition;
      inputs.positionsHealthy = healthy;
      inputs.resetOccurred = reset;
      reset = false;
    }
    public boolean setVelocity(double velocity, double feedForward) {
      velocityCalls++;
      this.velocity = velocity;
      this.feedForward = feedForward;
      return controlOk;
    }
    public boolean holdPosition(double position) {
      holdCalls++;
      target = position;
      return controlOk;
    }
    public void stop() { stopCalls++; }
  }
}
