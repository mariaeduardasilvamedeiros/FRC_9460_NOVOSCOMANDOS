package frc.robot.subsystems.manipulador;

import static frc.robot.subsystems.manipulador.ManipuladorConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.function.BooleanSupplier;

import org.junit.jupiter.api.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.manipulador.ManipuladorAbrirCommand;
import frc.robot.commands.manipulador.ManipuladorFecharCommand;
import frc.robot.commands.manipulador.ManipuladorGirarManualCommand;
import frc.robot.commands.manipulador.ManipuladorGirarParaCommand;
import frc.robot.commands.manipulador.ManipuladorTesteCommand;
import frc.robot.subsystems.manipulador.Manipulador.EstadoGarra;

/** Manipulador com a física da simulação: garra (motor + mola), rodas, rotação e segurança. */
class ManipuladorTest {

    // #region PARTE 1 - CONFIGURAÇÃO E PREPARAÇÃO

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private Manipulador manipulador;

    // #region PREPARAÇÃO DOS TESTES

    @BeforeAll static void hal() { assertTrue(HAL.initialize(500, 0)); }

    @BeforeEach void setup() {
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        withObject(0.0);
    }

    // #endregion

    // #endregion

    // #region PARTE 2 - MÉTODOS AUXILIARES

    /** Recria o manipulador com um objeto desta largura na garra simulada (0 = sem objeto). */
    private void withObject(double widthMeters) {
        if (manipulador != null) manipulador.close();
        manipulador = new Manipulador(new ManipuladorGarraIOSim(widthMeters),
            new ManipuladorRodasIOSim(), new ManipuladorGiroIOSim());
        cycle();
    }

    private void cycle() {
        SimHooks.stepTiming(0.02);
        scheduler.run();
    }

    /** Roda até a condição (ou o limite de ciclos); devolve quantos ciclos levou. */
    private int runUntil(BooleanSupplier done, int maxCycles) {
        for (int i = 0; i < maxCycles; i++) {
            if (done.getAsBoolean()) return i;
            cycle();
        }
        return maxCycles;
    }

    // #endregion

    // #region PARTE 3 - CASOS DE TESTE E ENCERRAMENTO

    @AfterEach void cleanup() {
        scheduler.cancelAll();
        manipulador.close();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
    }

    // #region GEOMETRIA DO CAD

    @Test void clawGeometryMatchesTheDrawing() {
        assertEquals(29.3, CAD_RIGHT_ARM_DEGREES, 0.1);
        assertEquals(25.5, CAD_LEFT_ARM_DEGREES, 0.1);
        assertEquals(4.17, CLAW_REDUCTION, 0.01);
        // As rodas se encostam com cada braço a ~5,4°; o limite para ~1,1 cm antes.
        assertEquals(5.4, WHEELS_TOUCH_ARM_DEGREES, 0.1);
        assertEquals(0.0, Manipulador.wheelGapMeters(CLAW_OPEN_ARM_DEGREES - WHEELS_TOUCH_ARM_DEGREES), 1e-9);
        assertEquals(0.011, Manipulador.wheelGapMeters(CLAW_CLOSED_LIMIT_DEGREES), 0.001);
        // Toda aberta (o 0): ~22 cm entre as rodas; no desenho eram 14,7 cm.
        assertEquals(0.220, Manipulador.wheelGapMeters(0.0), 0.002);
        assertEquals(0.147, Manipulador.wheelGapMeters(
            CLAW_OPEN_ARM_DEGREES - (CAD_RIGHT_ARM_DEGREES + CAD_LEFT_ARM_DEGREES) / 2.0), 0.002);
        assertEquals(33.0, CLAW_CLOSED_LIMIT_DEGREES, 1e-9);
    }

    // #endregion

    // #region MOTORES

    @Test void motorsAndIdsAreTheOnesOnTheMechanism() {
        // Informados pela equipe em 11/11/2026: Spark Flex 41 gira a garra,
        // X60 42 fecha a garra e X60 43 gira as rodas flexíveis.
        assertEquals(41, ROTATION_MOTOR_ID);
        assertEquals(42, CLAW_MOTOR_ID);
        assertEquals(43, WHEELS_MOTOR_ID);
        // A rotação é um NEO Vortex; garra e rodas são Kraken X60.
        assertEquals(edu.wpi.first.math.system.plant.DCMotor.getNeoVortex(1).KvRadPerSecPerVolt,
            ROTATION_SIM_MOTOR.KvRadPerSecPerVolt, 1e-9);
        assertEquals(edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1).KvRadPerSecPerVolt,
            CLAW_SIM_MOTOR.KvRadPerSecPerVolt, 1e-9);
        assertEquals(CLAW_SIM_MOTOR.KvRadPerSecPerVolt, WHEELS_SIM_MOTOR.KvRadPerSecPerVolt, 1e-9);
        // kV do NEO Vortex (6784 RPM a 12 V): ~0,106 V por volta/s do rotor.
        assertEquals(0.106, ROTATION_KV, 0.001);
    }

    @Test void theTwoTopMotorsSpinTheSameWayWhileClosingAndPulling() {
        // Os dois X60 ficam em cima, lado a lado: fechar (garra) e puxar (rodas)
        // giram os dois rotores para o mesmo lado, o positivo.
        assertEquals(CLAW_INVERTED, WHEELS_INVERTED);
        var garra = new ManipuladorGarraIOSim(0.0);
        var rodas = new ManipuladorRodasIOSim();
        manipulador.close();
        manipulador = new Manipulador(garra, rodas, new ManipuladorGiroIOSim());
        cycle();
        scheduler.schedule(new ManipuladorFecharCommand(manipulador));
        var clawInputs = new ManipuladorGarraIO.ManipuladorGarraIOInputs();
        var wheelsInputs = new ManipuladorRodasIO.ManipuladorRodasIOInputs();
        double clawPeak = 0.0;
        double wheelsPeak = 0.0;
        for (int i = 0; i < 8; i++) {
            cycle();
            clawPeak = Math.max(clawPeak, manipulador.getClawClosingDegrees());
            wheelsPeak = Math.max(wheelsPeak, manipulador.getWheelsRpm());
        }
        assertTrue(clawPeak > 5.0, "garra fechando: rotor no sentido positivo (" + clawPeak + "°)");
        assertTrue(wheelsPeak > 500.0, "rodas puxando: rotor no sentido positivo (" + wheelsPeak + " RPM)");
        garra.updateInputs(clawInputs);
        rodas.updateInputs(wheelsInputs);
        assertTrue(clawInputs.positionRotations > 0.0 && wheelsInputs.positionRotations > 0.0,
            "os dois rotores andaram para o mesmo lado");
    }

    // #endregion

    // #region GARRA

    @Test void withoutAnObjectTheClawClosesToTheLimitAndTheWheelsKeepPullingAFewSeconds() {
        var fechar = new ManipuladorFecharCommand(manipulador);
        scheduler.schedule(fechar);
        double maxClosing = 0.0;
        int closedAt = -1;
        int cycles = 0;
        double wheelsRpmAfterClosing = 0.0;
        while (cycles++ < 200) {
            cycle();
            maxClosing = Math.max(maxClosing, manipulador.getClawClosingDegrees());
            if (closedAt < 0 && manipulador.isClawClosed()) closedAt = cycles;
            if (closedAt > 0 && cycles == closedAt + 50) wheelsRpmAfterClosing = manipulador.getWheelsRpm();
            if (!fechar.isScheduled()) break;
        }
        assertTrue(closedAt > 0 && closedAt < 40, "fechou em " + closedAt + " ciclos");
        assertEquals(EstadoGarra.FECHADA_VAZIA, manipulador.getClawState());
        assertFalse(manipulador.hasObject());
        // Com só 4,17:1 os braços são leves: o kD não deixa passar do limite ao chegar.
        assertTrue(maxClosing <= CLAW_CLOSED_LIMIT_DEGREES + 1.0, "passou do limite: " + maxClosing);
        // As rodas seguem puxando depois de fechar e o comando termina sozinho.
        assertTrue(wheelsRpmAfterClosing > 1000.0, "rodas puxando 1 s depois de fechar: " + wheelsRpmAfterClosing);
        assertFalse(fechar.isScheduled(), "terminou sozinho");
        assertEquals(WHEELS_PULL_AFTER_CLOSE_SECONDS / 0.02, cycles - closedAt, 3.0);
        // Fechada, continua fechada (sem comando rodando) e as rodas param.
        for (int i = 0; i < 50; i++) cycle();
        assertEquals(CLAW_CLOSED_LIMIT_DEGREES, manipulador.getClawClosingDegrees(), CLAW_POSITION_TOLERANCE_DEGREES);
        assertTrue(manipulador.getClawClosingDegrees() < CLAW_OPEN_ARM_DEGREES - WHEELS_TOUCH_ARM_DEGREES,
            "não esmaga as rodas uma na outra");
        assertEquals(0.0, manipulador.getWheelsRpm(), 50.0);
    }

    @Test void anObjectStopsTheClawAndItsWidthIsMeasured() {
        withObject(0.10);
        scheduler.schedule(new ManipuladorFecharCommand(manipulador));
        runUntil(manipulador::isClawClosed, 100);
        assertEquals(EstadoGarra.SEGURANDO_OBJETO, manipulador.getClawState());
        assertTrue(manipulador.hasObject());
        assertEquals(0.10, manipulador.getObjectWidthMeters(), 0.005, "largura medida pelo ângulo");
        // Segurando com tensão fixa: o objeto não escapa nem a garra fecha mais.
        double closing = manipulador.getClawClosingDegrees();
        for (int i = 0; i < 50; i++) cycle();
        assertEquals(EstadoGarra.SEGURANDO_OBJETO, manipulador.getClawState());
        assertEquals(closing, manipulador.getClawClosingDegrees(), 0.2);
    }

    @Test void releasingTheMotorLetsTheSpringOpenTheClaw() {
        scheduler.schedule(new ManipuladorFecharCommand(manipulador));
        runUntil(manipulador::isClawClosed, 100);
        var abrir = new ManipuladorAbrirCommand(manipulador);
        scheduler.schedule(abrir);
        int cycles = runUntil(() -> !abrir.isScheduled(), 100);
        assertTrue(cycles < 50, "a mola abriu em " + cycles + " ciclos");
        assertEquals(EstadoGarra.ABERTA, manipulador.getClawState());
        assertEquals(0.0, manipulador.getClawClosingDegrees(), CLAW_POSITION_TOLERANCE_DEGREES);
    }

    // #endregion

    // #region ROTAÇÃO E TESTE COMPLETO

    @Test void rotateCommandReachesTheAngleAndHoldsTheClawClosed() {
        withObject(0.10);
        scheduler.schedule(new ManipuladorFecharCommand(manipulador));
        runUntil(manipulador::isClawClosed, 100);
        var girar = new ManipuladorGirarParaCommand(manipulador, 90.0);
        scheduler.schedule(girar);
        int cycles = runUntil(() -> !girar.isScheduled(), 150);
        assertTrue(cycles < 100, "chegou em " + cycles + " ciclos");
        assertEquals(90.0, manipulador.getRotationDegrees(), ROTATION_TOLERANCE_DEGREES);
        // Girar não solta o objeto.
        assertTrue(manipulador.hasObject());
        for (int i = 0; i < 25; i++) cycle();
        assertEquals(90.0, manipulador.getRotationDegrees(), ROTATION_TOLERANCE_DEGREES);
    }

    @Test void fullTestRunsEveryStepWithTheSeparateCommands() {
        var teste = new ManipuladorTesteCommand(manipulador);
        scheduler.schedule(teste);
        double maxAngle = 0.0;
        boolean closed = false;
        for (int i = 0; i < 500 && teste.isScheduled(); i++) {
            cycle();
            maxAngle = Math.max(maxAngle, manipulador.getRotationDegrees());
            closed |= manipulador.isClawClosed();
        }
        assertFalse(teste.isScheduled(), "o teste terminou");
        assertEquals("OK", teste.getResultado());
        assertTrue(closed, "a garra fechou");
        assertEquals(TEST_ROTATION_DEGREES, maxAngle, 3.0);
        assertEquals(0.0, manipulador.getRotationDegrees(), 3.0);
        assertEquals(EstadoGarra.ABERTA, manipulador.getClawState(), "terminou com a garra aberta");
    }

    @Test void releasingTheButtonStopsTheTestAndOpensTheClaw() {
        var teste = new ManipuladorTesteCommand(manipulador);
        scheduler.schedule(teste);
        runUntil(manipulador::isClawClosed, 100);
        for (int i = 0; i < 35; i++) cycle();
        scheduler.cancel(teste);
        assertTrue(teste.getResultado().startsWith("INTERROMPIDO_EM_"), teste.getResultado());
        double angle = manipulador.getRotationDegrees();
        for (int i = 0; i < 40; i++) cycle();
        assertEquals(EstadoGarra.ABERTA, manipulador.getClawState(), "a mola abriu");
        assertEquals(angle, manipulador.getRotationDegrees(), 15.0, "rotação parou perto de onde estava");
    }

    // #endregion

    // #region SEGURANÇA

    @Test void rotationNeverPassesTheWiringLimit() {
        double max = 0.0;
        double min = 0.0;
        for (int i = 0; i < 100; i++) {
            manipulador.setRotationDegrees(200.0);
            cycle();
            max = Math.max(max, manipulador.getRotationDegrees());
        }
        assertEquals(ROTATION_END_DEGREES, manipulador.getRotationDegrees(), ROTATION_TOLERANCE_DEGREES);
        for (int i = 0; i < 100; i++) {
            manipulador.setRotationDegrees(-200.0);
            cycle();
            min = Math.min(min, manipulador.getRotationDegrees());
        }
        assertEquals(0.0, manipulador.getRotationDegrees(), ROTATION_TOLERANCE_DEGREES);
        assertTrue(max <= ROTATION_MAX_DEGREES + ROTATION_HARD_STOP_MARGIN_DEGREES, "passou da fiação: " + max);
        assertTrue(min >= ROTATION_MIN_DEGREES - ROTATION_HARD_STOP_MARGIN_DEGREES, "passou da fiação: " + min);
    }

    @Test void disablingOpensTheClawStopsEverythingAndNothingRestartsAlone() {
        var rotation = new CountingRotationIOSim();
        manipulador.close();
        manipulador = new Manipulador(new ManipuladorGarraIOSim(0.0), new ManipuladorRodasIOSim(), rotation);
        cycle();
        scheduler.schedule(new ManipuladorFecharCommand(manipulador));
        runUntil(manipulador::isClawClosed, 100);
        manipulador.setRotationDegrees(90.0);
        for (int i = 0; i < 10; i++) cycle();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        for (int i = 0; i < 50; i++) cycle();
        assertEquals(EstadoGarra.ABERTA, manipulador.getClawState(), "desabilitado: a mola abre");
        int targets = rotation.targetCommands;
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        for (int i = 0; i < 25; i++) cycle();
        assertEquals(EstadoGarra.ABERTA, manipulador.getClawState(), "habilitar não fecha sozinho");
        // Com o motor direto o manipulador pode seguir girando por inércia; o que
        // importa é que nenhuma meta antiga volta a ser pedida ao motor.
        assertEquals(targets, rotation.targetCommands, "habilitar não retoma a meta antiga");
        assertFalse(manipulador.isRotationAtGoal());
        manipulador.setRotationDegrees(Double.NaN);
        assertFalse(manipulador.isRotationAtGoal());
    }

    @Test void upTurnsToTheMeasuredEndAndDownReturnsToZero() {
        // Medido no REV Hardware Client: encoder de 0,02846 (posição 0) a 0,280134
        // (fim). Curso = 0,251674 volta = 90,6° com 1:1.
        // A equipe pediu para ir 5° além: fim = 90,6° + 5° = 95,6°.
        assertEquals(1.0, ROTATION_REDUCTION);
        assertEquals(0.280134 - 0.02846, ROTATION_MEASURED_END_MOTOR_ROTATIONS, 1e-9);
        assertEquals(90.6 + 5.0, ROTATION_TURNED_DEGREES, 0.01);
        assertTrue(ROTATION_MAX_DEGREES > ROTATION_TURNED_DEGREES && ROTATION_MIN_DEGREES < 0.0);
        var up = new ManipuladorGirarParaCommand(manipulador, ROTATION_TURNED_DEGREES);
        scheduler.schedule(up);
        int cycles = runUntil(() -> !up.isScheduled(), 200);
        assertTrue(cycles * 0.02 < ROTATION_MOVE_TIMEOUT_SECONDS, "chegou em " + cycles * 0.02 + " s");
        assertEquals(ROTATION_TURNED_DEGREES, manipulador.getRotationDegrees(), ROTATION_TOLERANCE_DEGREES);
        var down = new ManipuladorGirarParaCommand(manipulador, 0.0);
        scheduler.schedule(down);
        double max = manipulador.getRotationDegrees();
        for (int i = 0; i < 200 && down.isScheduled(); i++) {
            cycle();
            max = Math.max(max, manipulador.getRotationDegrees());
        }
        assertFalse(down.isScheduled(), "voltou ao 0");
        assertEquals(0.0, manipulador.getRotationDegrees(), ROTATION_TOLERANCE_DEGREES);
        assertTrue(max <= ROTATION_MAX_DEGREES, "não passou do soft limit: " + max);
    }

    @Test void releasingTheRotateButtonMidwayBrakesInsteadOfCoasting() {
        var up = new ManipuladorGirarParaCommand(manipulador, ROTATION_TURNED_DEGREES);
        scheduler.schedule(up);
        for (int i = 0; i < 25; i++) cycle(); // 0,5 s: no meio do giro, em velocidade.
        double atRelease = manipulador.getRotationDegrees();
        assertTrue(atRelease > 10.0 && atRelease < ROTATION_TURNED_DEGREES - 20.0, "no meio: " + atRelease);
        scheduler.cancel(up);
        for (int i = 0; i < 150; i++) cycle();
        double stopped = manipulador.getRotationDegrees();
        // Freia pelo perfil (126°/s a 216°/s² para em ~37°) e segura; sem isso,
        // o manipulador seguiria por inércia até o batente.
        assertTrue(stopped < ROTATION_TURNED_DEGREES - 10.0, "parou antes do fim: " + stopped);
        for (int i = 0; i < 50; i++) cycle();
        assertEquals(stopped, manipulador.getRotationDegrees(), 0.5, "segura onde parou");
    }

    @Test void realMotorConfigurationsUseTheSafetyLimits() {
        var claw = ManipuladorGarraIOTalonFX.configuration();
        assertEquals(com.ctre.phoenix6.signals.NeutralModeValue.Coast, claw.MotorOutput.NeutralMode,
            "solta em Coast: a mola abre");
        assertEquals(CLAW_STATOR_CURRENT_LIMIT_AMPS, claw.CurrentLimits.StatorCurrentLimit);
        assertTrue(claw.CurrentLimits.StatorCurrentLimitEnable && claw.CurrentLimits.SupplyCurrentLimitEnable);
        assertEquals(com.ctre.phoenix6.signals.StaticFeedforwardSignValue.UseClosedLoopSign,
            claw.Slot0.StaticFeedforwardSign, "kS como na simulação");
        // Soft limit de fechar: além da meta e antes das rodas se encostarem.
        double softLimitDegrees = claw.SoftwareLimitSwitch.ForwardSoftLimitThreshold / CLAW_REDUCTION * 360.0;
        assertTrue(softLimitDegrees > CLAW_CLOSED_LIMIT_DEGREES, "soft limit além da meta: " + softLimitDegrees);
        assertTrue(softLimitDegrees < CLAW_OPEN_ARM_DEGREES - WHEELS_TOUCH_ARM_DEGREES,
            "soft limit antes das rodas se encostarem: " + softLimitDegrees);
        assertTrue(claw.SoftwareLimitSwitch.ForwardSoftLimitEnable && claw.SoftwareLimitSwitch.ReverseSoftLimitEnable);
        // O aperto parado (V/R) fica abaixo do limite de corrente da garra.
        assertTrue(CLAW_HOLD_VOLTS / CLAW_SIM_MOTOR.rOhms < CLAW_STATOR_CURRENT_LIMIT_AMPS,
            "aperto de " + CLAW_HOLD_VOLTS / CLAW_SIM_MOTOR.rOhms + " A");

        var wheels = ManipuladorRodasIOTalonFX.configuration();
        assertEquals(com.ctre.phoenix6.signals.NeutralModeValue.Brake, wheels.MotorOutput.NeutralMode);
        assertEquals(WHEELS_STATOR_CURRENT_LIMIT_AMPS, wheels.CurrentLimits.StatorCurrentLimit);
        assertTrue(wheels.CurrentLimits.StatorCurrentLimitEnable && wheels.CurrentLimits.SupplyCurrentLimitEnable);
        // Os dois X60 de cima giram para o mesmo lado.
        assertEquals(claw.MotorOutput.Inverted, wheels.MotorOutput.Inverted);
    }

    @Test void clawThatTakesTooLongToCloseIsReleasedUntilANewRequest() {
        var claw = new CreepingClawIO();
        manipulador.close();
        manipulador = new Manipulador(claw, new ManipuladorRodasIOSim(), new ManipuladorGiroIOSim());
        cycle();
        var fechar = new ManipuladorFecharCommand(manipulador);
        scheduler.schedule(fechar);
        int cycles = runUntil(() -> !"NENHUMA".equals(manipulador.getClawFault()), 150);
        assertEquals("TEMPO_ESGOTADO_AO_FECHAR", manipulador.getClawFault());
        assertEquals(CLAW_CLOSE_TIMEOUT_SECONDS / 0.02, cycles, 3.0, "soltou no tempo máximo");
        assertFalse(manipulador.isClawClosed());
        // O comando segue agendado, mas a garra não volta a fechar sozinha.
        int closeCommands = claw.closeCommands;
        for (int i = 0; i < 25; i++) cycle();
        assertEquals(closeCommands, claw.closeCommands, "não fecha de novo sem novo pedido");
        assertTrue(fechar.isScheduled());
        // Apertar → de novo: novo pedido, fecha outra vez.
        scheduler.cancel(fechar);
        scheduler.schedule(new ManipuladorFecharCommand(manipulador));
        for (int i = 0; i < 5; i++) cycle();
        assertEquals("NENHUMA", manipulador.getClawFault());
        assertTrue(claw.closeCommands > closeCommands, "voltou a fechar");
    }

    @Test void rotationThatNeverArrivesStopsUntilTheButtonIsReleased() {
        var rotation = new StuckRotationIO();
        manipulador.close();
        manipulador = new Manipulador(new ManipuladorGarraIOSim(0.0), new ManipuladorRodasIOSim(), rotation);
        cycle();
        var girar = new ManipuladorGirarParaCommand(manipulador, 90.0);
        scheduler.schedule(girar);
        int cycles = runUntil(() -> "TEMPO_ESGOTADO".equals(
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getString("Manipulador/Giro/Estado", "")), 250);
        assertEquals(ROTATION_MOVE_TIMEOUT_SECONDS / 0.02, cycles, 3.0, "parou no tempo máximo");
        assertTrue(rotation.stopped, "motor do giro parado");
        int targets = rotation.targetCommands;
        for (int i = 0; i < 25; i++) cycle();
        assertEquals(targets, rotation.targetCommands, "não tenta girar de novo com o botão segurado");
        assertFalse(manipulador.isRotationAtGoal());
        // Soltar o botão (comando interrompido) libera; apertar de novo tenta outra vez.
        scheduler.cancel(girar);
        scheduler.schedule(new ManipuladorGirarParaCommand(manipulador, 90.0));
        cycle();
        assertTrue(rotation.targetCommands > targets, "tentou girar de novo");
    }

    /** Garra que fecha devagar demais (6°/s) e nunca para: não prende nem chega ao limite. */
    private static final class CreepingClawIO implements ManipuladorGarraIO {
        double closingDegrees;
        boolean closing;
        int closeCommands;

        @Override public void setClosingTargetRotations(double motorRotations) {
            closing = true;
            closeCommands++;
        }
        @Override public void holdWithVolts(double volts) { closing = true; }
        @Override public void release() { closing = false; }
        @Override public void updateInputs(ManipuladorGarraIOInputs inputs) {
            closingDegrees = closing ? closingDegrees + 0.12 : Math.max(0.0, closingDegrees - 2.0);
            inputs.connected = true;
            inputs.zeroValid = true;
            inputs.positionRotations = closingDegrees / 360.0 * CLAW_REDUCTION;
            inputs.velocityRotationsPerSec = (closing ? 6.0 : -100.0) / 360.0 * CLAW_REDUCTION;
        }
    }

    @Test void heldDirectionUsesPositionProfileToTheEndAndBack() {
        var up = new ManipuladorGirarManualCommand(manipulador, +1);
        scheduler.schedule(up);
        double max = 0.0;
        double maxSpeed = 0.0;
        double previous = manipulador.getRotationDegrees();
        int reachedAt = -1;
        for (int i = 0; i < 250; i++) {
            cycle();
            double angle = manipulador.getRotationDegrees();
            max = Math.max(max, angle);
            maxSpeed = Math.max(maxSpeed, (angle - previous) / 0.02);
            previous = angle;
            if (reachedAt < 0 && angle >= ROTATION_END_DEGREES - 5.0) reachedAt = i;
        }
        assertTrue(reachedAt >= 0, "chegou perto do fim: " + max);
        assertTrue(max <= ROTATION_MAX_DEGREES, "não passou do soft limit: " + max);
        assertTrue(maxSpeed <= ROTATION_CRUISE_ROTATIONS_PER_SEC / ROTATION_REDUCTION * 360.0 + 10.0, "velocidade máxima: " + maxSpeed);
        scheduler.cancel(up); // Soltou ↑.

        var down = new ManipuladorGirarManualCommand(manipulador, -1);
        scheduler.schedule(down);
        double min = manipulador.getRotationDegrees();
        for (int i = 0; i < 250; i++) {
            cycle();
            min = Math.min(min, manipulador.getRotationDegrees());
        }
        assertEquals(0.0, manipulador.getRotationDegrees(), 3.0, "voltou ao 0");
        assertTrue(min >= ROTATION_MIN_DEGREES, "não passou do soft limit do 0: " + min);
    }

    @Test void releasingManualRotationBrakesAndHoldsWithoutMovingTheTargetAgain() {
        var up = new ManipuladorGirarManualCommand(manipulador, +1);
        scheduler.schedule(up);
        for (int i = 0; i < 25; i++) cycle();
        double released = manipulador.getRotationDegrees();
        assertTrue(released > 10.0 && released < ROTATION_END_DEGREES - 20.0);
        scheduler.cancel(up);
        double target = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber(
            "Manipulador/Giro/MetaGraus", Double.NaN);
        // BACK mantido nao pode recalcular o alvo a cada ciclo e reiniciar o timeout.
        cycle();
        target = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber(
            "Manipulador/Giro/MetaGraus", Double.NaN);
        for (int i = 0; i < 150; i++) { manipulador.stopAll(); cycle(); }
        double stopped = manipulador.getRotationDegrees();
        assertTrue(stopped > released && stopped < ROTATION_END_DEGREES - 10.0,
            "freou antes do fim: " + stopped);
        assertEquals(target, edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber(
            "Manipulador/Giro/MetaGraus", Double.NaN), 1e-9);
        for (int i = 0; i < 50; i++) cycle();
        assertEquals(stopped, manipulador.getRotationDegrees(), 0.5);
        assertFalse(manipulador.hasRotationFault());
    }

    @Test void stalledManualRotationStopsAndRequiresANewButtonPress() {
        var rotation = new StuckRotationIO();
        manipulador.close();
        manipulador = new Manipulador(new ManipuladorGarraIOSim(0), new ManipuladorRodasIOSim(), rotation);
        cycle();
        var command = new ManipuladorGirarManualCommand(manipulador, +1);
        scheduler.schedule(command);
        runUntil(manipulador::hasRotationFault, 200);
        assertEquals("TEMPO_ESGOTADO", manipulador.getRotationFault());
        assertTrue(rotation.stopped);
        assertFalse(command.isScheduled());
        int count = rotation.targetCommands;
        for (int i = 0; i < 50; i++) { manipulador.rotateManual(1); manipulador.holdRotationHere(); cycle(); }
        assertEquals(count, rotation.targetCommands, "falha nao pode ser apagada pelo ciclo de controle");
        scheduler.schedule(command);
        cycle();
        assertTrue(rotation.targetCommands > count);
        assertFalse(manipulador.hasRotationFault());
    }

    @Test void invalidVelocityInterruptsMovementAndDoesNotResumeOnSensorRecovery() {
        var rotation = new StuckRotationIO();
        manipulador.close();
        manipulador = new Manipulador(new ManipuladorGarraIOSim(0), new ManipuladorRodasIOSim(), rotation);
        cycle();
        scheduler.schedule(new ManipuladorGirarManualCommand(manipulador, +1));
        cycle();
        rotation.velocity = Double.NaN;
        cycle();
        assertEquals("LEITURA_INVALIDA_REFACA_ZERO", manipulador.getRotationFault());
        assertTrue(rotation.stopped);
        int count = rotation.targetCommands;
        rotation.velocity = 0.0;
        for (int i = 0; i < 10; i++) cycle();
        assertEquals(count, rotation.targetCommands);
        assertTrue(manipulador.hasRotationFault());
    }

    @Test void rotationWithoutPhysicalReferenceRejectsMovement() {
        var rotation = new StuckRotationIO();
        rotation.zeroValid = false;
        manipulador.close();
        manipulador = new Manipulador(new ManipuladorGarraIOSim(0), new ManipuladorRodasIOSim(), rotation);
        cycle();
        scheduler.schedule(new ManipuladorGirarManualCommand(manipulador, +1));
        for (int i = 0; i < 20; i++) cycle();
        assertEquals(0, rotation.targetCommands);
        assertTrue(rotation.stopped);
    }

    @Test void passingThroughTheGoalAtSpeedDoesNotCountAsArrived() {
        var rotation = new StuckRotationIO();
        manipulador.close();
        manipulador = new Manipulador(new ManipuladorGarraIOSim(0), new ManipuladorRodasIOSim(), rotation);
        cycle();
        manipulador.setRotationDegrees(0.0);
        rotation.velocity = 30.0 / 360.0 * ROTATION_REDUCTION;
        cycle();
        assertFalse(manipulador.isRotationAtGoal());
        rotation.velocity = 0.0;
        cycle();
        assertTrue(manipulador.isRotationAtGoal());
    }

    @Test void directionReversalKeepsTheMechanismWithinTravel() {
        var up = new ManipuladorGirarManualCommand(manipulador, +1);
        scheduler.schedule(up);
        for (int i = 0; i < 25; i++) cycle();
        scheduler.schedule(new ManipuladorGirarManualCommand(manipulador, -1));
        for (int i = 0; i < 200; i++) {
            cycle();
            assertTrue(manipulador.getRotationDegrees() >= ROTATION_MIN_DEGREES);
            assertTrue(manipulador.getRotationDegrees() <= ROTATION_MAX_DEGREES);
        }
        assertTrue(manipulador.isRotationAtDegrees(0.0));
        assertFalse(manipulador.hasRotationFault());
    }

    @Test void motorsTurnedOffInTheConstantsNeverMove() {
        manipulador.close();
        manipulador = new Manipulador(ManipuladorGarraIO.desligado(), ManipuladorRodasIO.desligado(),
            ManipuladorGiroIO.desligado());
        cycle();
        scheduler.schedule(new ManipuladorFecharCommand(manipulador));
        manipulador.setRotationDegrees(ROTATION_TURNED_DEGREES);
        for (int i = 0; i < 25; i++) cycle();
        assertFalse(manipulador.isClawClosed());
        assertEquals(0.0, manipulador.getWheelsRpm());
        assertFalse(manipulador.isRotationAtGoal());
        assertEquals("DESLIGADO_NAS_CONSTANTES",
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getString("Manipulador/Giro/Zero", ""));
        assertEquals("DESLIGADO_NAS_CONSTANTES",
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getString("Manipulador/Rodas/Configuracao", ""));
    }

    /** Giro simulado que conta as metas pedidas ao motor. */
    private static final class CountingRotationIOSim extends ManipuladorGiroIOSim {
        int targetCommands;

        @Override public void setTargetRotations(double motorRotations) {
            targetCommands++;
            super.setTargetRotations(motorRotations);
        }
    }

    /** Giro preso: aceita a meta, mas nunca sai do lugar. */
    private static final class StuckRotationIO implements ManipuladorGiroIO {
        int targetCommands;
        boolean stopped = true;
        boolean zeroValid = true;
        double velocity;

        @Override public void setTargetRotations(double motorRotations) {
            targetCommands++;
            stopped = false;
        }
        @Override public void stop() { stopped = true; }
        @Override public void updateInputs(ManipuladorGiroIOInputs inputs) {
            inputs.connected = true;
            inputs.zeroValid = zeroValid;
            inputs.velocityRotationsPerSec = velocity;
            inputs.positionRotations = 0.0;
        }
    }

    // #endregion

    // #region DESENHO 3D

    @Test void componentPosesFollowTheJointChain() {
        Pose3d[] zero = ManipuladorVisualizer.componentPoses(0.0, 0.0, 0.0, false);
        assertEquals(6, zero.length);
        // No 0 (toda aberta): cada braço no próprio pivô, aberto além do desenho.
        assertEquals(PIVOT_RIGHT_Y_METERS, zero[1].getY(), 1e-9);
        assertEquals(ROLL_AXIS_HEIGHT_METERS, zero[1].getZ(), 1e-9);
        assertEquals(Math.toRadians(CAD_RIGHT_ARM_DEGREES - CLAW_OPEN_ARM_DEGREES), zero[1].getRotation().getZ(), 1e-9);
        assertEquals(Math.toRadians(CLAW_OPEN_ARM_DEGREES - CAD_LEFT_ARM_DEGREES), zero[2].getRotation().getZ(), 1e-9);
        // A roda direita a 40° do eixo da garra, à direita (Y negativo).
        double wheelAngle = Math.toDegrees(Math.atan2(PIVOT_RIGHT_Y_METERS - zero[3].getY(), zero[3].getX() - PIVOT_X_METERS));
        assertEquals(CLAW_OPEN_ARM_DEGREES, wheelAngle, 0.01);
        assertTrue(zero[5].getZ() < 0.0, "sem objeto: fica escondido");

        // Fechando, o braço direito gira para a esquerda e o esquerdo para a direita.
        Pose3d[] closed = ManipuladorVisualizer.componentPoses(0.0, 10.0, 0.0, true);
        assertEquals(zero[1].getRotation().getZ() + Math.toRadians(10.0), closed[1].getRotation().getZ(), 1e-9);
        assertEquals(zero[2].getRotation().getZ() - Math.toRadians(10.0), closed[2].getRotation().getZ(), 1e-9);
        assertTrue(closed[3].getY() > zero[3].getY(), "a roda direita foi para o centro");
        assertEquals(ROLL_AXIS_HEIGHT_METERS, closed[5].getZ(), 1e-9);

        // Girou 90°: o pivô direito (à direita do eixo) vai para baixo do eixo.
        Pose3d[] rolled = ManipuladorVisualizer.componentPoses(90.0, 0.0, 0.0, false);
        assertEquals(0.0, rolled[1].getY(), 1e-9);
        assertEquals(ROLL_AXIS_HEIGHT_METERS + PIVOT_RIGHT_Y_METERS, rolled[1].getZ(), 1e-9);
    }

    @Test void the3dModelPivotsMatchTheConstants() throws Exception {
        // Gerado por config/advantagescope/scripts/preparar_manipulador.py a partir do CAD.
        var config = new com.fasterxml.jackson.databind.ObjectMapper().readTree(java.nio.file.Path.of(
            "config/advantagescope/mecanismos/Robot_Manipulador_9460/config.json").toFile());
        assertEquals("FRC9460 - Manipulador", config.get("name").asText());
        var position = config.get("position");
        var components = config.get("components");
        assertEquals(6, components.size(), "corpo, 2 braços, 2 rodas e o objeto");
        double[][] pivots = {
            {0.0, 0.0, ROLL_AXIS_HEIGHT_METERS},
            {PIVOT_X_METERS, PIVOT_RIGHT_Y_METERS, ROLL_AXIS_HEIGHT_METERS},
            {PIVOT_X_METERS, PIVOT_LEFT_Y_METERS, ROLL_AXIS_HEIGHT_METERS},
            {AXLE_RIGHT_X_METERS, AXLE_RIGHT_Y_METERS, ROLL_AXIS_HEIGHT_METERS},
            {AXLE_LEFT_X_METERS, AXLE_LEFT_Y_METERS, ROLL_AXIS_HEIGHT_METERS}};
        for (int i = 0; i < pivots.length; i++) {
            var zeroed = components.get(i).get("zeroedPosition");
            for (int axis = 0; axis < 3; axis++) {
                // zeroedPosition = posição do modelo - pivô (regra do preparar_componentes).
                assertEquals(pivots[i][axis], position.get(axis).asDouble() - zeroed.get(axis).asDouble(), 2e-4,
                    "componente " + i + ", eixo " + axis);
            }
        }
    }

    // #endregion

    // #endregion
}
