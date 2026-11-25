package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import java.util.HashSet;
import java.util.Locale;

import org.junit.jupiter.api.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.swerve.SwerveDirigirCommand;
import frc.robot.subsystems.elevatorpivot.ElevatorPivotConstants;
import frc.robot.subsystems.manipulador.ManipuladorConstants;

/** Swerve com a física da simulação: medidas, pilotagem orientada ao campo, alianças e segurança. */
class SwerveTest {

    // #region PARTE 1 - CONFIGURAÇÃO E PREPARAÇÃO

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private Swerve swerve;
    private SwerveGyroIOSim gyro;
    private SwerveIOSim[] modules;

    // Analógicos do piloto, já no sentido do comando (+ = frente, esquerda, anti-horário).
    private double forward;
    private double left;
    private double counterClockwise;
    private boolean slow;

    // #region PREPARAÇÃO DOS TESTES

    @BeforeAll static void hal() { assertTrue(HAL.initialize(500, 0)); }

    @BeforeEach void setup() {
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        forward = 0.0;
        left = 0.0;
        counterClockwise = 0.0;
        slow = false;
        create(new SwerveGyroIOSim());
    }

    // #endregion

    // #endregion

    // #region PARTE 2 - MÉTODOS AUXILIARES

    /** Recria o swerve com módulos simulados, este giroscópio e o comando de pilotagem. */
    private void create(SwerveGyroIO gyroIO) {
        if (swerve != null) swerve.close();
        gyro = gyroIO instanceof SwerveGyroIOSim sim ? sim : null;
        modules = new SwerveIOSim[] {new SwerveIOSim(), new SwerveIOSim(), new SwerveIOSim(), new SwerveIOSim()};
        swerve = new Swerve(gyroIO, modules[0], modules[1], modules[2], modules[3]);
        swerve.setDefaultCommand(new SwerveDirigirCommand(swerve, () -> forward, () -> left,
            () -> counterClockwise, () -> slow));
        cycle();
    }

    /**
     * O padrão do piloto agora é 50%. Os testes de direção e giro foram escritos com o
     * analógico a 100%: este método põe o swerve atual em 100%. Chamar uma vez por swerve
     * (de novo se o teste recriar o swerve).
     */
    private void fullSpeed() {
        swerve.toggleSpeedScale(1.0);
    }

    private void cycle() {
        SimHooks.stepTiming(0.02);
        scheduler.run();
    }

    private void cycles(int count) {
        for (int i = 0; i < count; i++) cycle();
    }

    private double speedMetersPerSec() {
        var speeds = swerve.getRobotRelativeSpeeds();
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    private double gyroYawDegrees() {
        var inputs = new SwerveGyroIO.SwerveGyroIOInputs();
        gyro.updateInputs(inputs);
        return Math.toDegrees(inputs.yawRad);
    }

    /** Roda apontada neste ângulo, aceitando também a roda invertida (otimização). */
    private static void assertWheelAngle(double expectedDegrees, Rotation2d actual, String name) {
        double error = MathUtil.inputModulus(actual.getDegrees() - expectedDegrees, -90.0, 90.0);
        assertEquals(0.0, error, 2.0, name + " em " + actual.getDegrees() + "°");
    }

    private void setAlliance(AllianceStationID station) {
        DriverStationSim.setAllianceStationId(station);
        DriverStationSim.notifyNewData();
    }

    // #endregion

    // #region PARTE 3 - CASOS DE TESTE E ENCERRAMENTO

    @AfterEach void cleanup() {
        scheduler.cancelAll();
        swerve.close();
        swerve = null;
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
    }

    // #region MEDIDAS E MOTORES

    @Test void canIdsAreValidAndDoNotRepeat() {
        // Swerve (8 Spark MAX + Pigeon 2) e os outros mecanismos (pivô 38/39, manipulador 41/42/43).
        int[] ids = {FE_DRIVE_ID, FE_TURN_ID, FD_DRIVE_ID, FD_TURN_ID, TE_DRIVE_ID, TE_TURN_ID,
            TD_DRIVE_ID, TD_TURN_ID, GYRO_ID,
            ElevatorPivotConstants.kLeaderId, ElevatorPivotConstants.kFollowerId,
            ManipuladorConstants.ROTATION_MOTOR_ID, ManipuladorConstants.CLAW_MOTOR_ID,
            ManipuladorConstants.WHEELS_MOTOR_ID};
        var all = new HashSet<Integer>();
        for (int id : ids) {
            assertTrue(id >= 0 && id <= 62, "ID " + id + " fora de 0 a 62");
            assertTrue(all.add(id), "ID " + id + " repetido");
        }
    }

    @Test void chassisIs26AndAHalfInchesWithWheels23InchesApart() {
        assertEquals(Units.inchesToMeters(23.0), WHEEL_BASE_METERS, 1e-9);
        assertEquals(Units.inchesToMeters(23.0), TRACK_WIDTH_METERS, 1e-9);
        // MAXSwerve: centro da roda a 1,75" da borda do chassi de 26,5".
        assertEquals(Units.inchesToMeters(1.75), (FRAME_SIZE_METERS - WHEEL_BASE_METERS) / 2.0, 1e-9);
        assertEquals(0.4131, DRIVE_BASE_RADIUS_METERS, 1e-3);
        // X frente, Y esquerda: FE (+,+), FD (+,−), TE (−,+), TD (−,−).
        assertTrue(FE_LOCATION.getX() > 0 && FE_LOCATION.getY() > 0);
        assertTrue(FD_LOCATION.getX() > 0 && FD_LOCATION.getY() < 0);
        assertTrue(TE_LOCATION.getX() < 0 && TE_LOCATION.getY() > 0);
        assertTrue(TD_LOCATION.getX() < 0 && TD_LOCATION.getY() < 0);
    }

    @Test void maxSwerveThreeInchReductionFreeSpeedAndLimits() {
        // Os três pinhões do kit REV-21-3005 com a engrenagem de 22 dentes (fórmula REV).
        assertEquals(0.0762, WHEEL_DIAMETER_METERS, 1e-9);
        assertEquals(5.50, driveReduction(12, 22), 0.005);
        assertEquals(5.08, driveReduction(13, 22), 0.005);
        assertEquals(4.71, driveReduction(14, 22), 0.005);
        double neoWheelMetersPerSec = NEO_FREE_SPEED_RPM / 60.0 * Math.PI * WHEEL_DIAMETER_METERS;
        assertEquals(4.12, neoWheelMetersPerSec / driveReduction(12, 22), 0.01);
        assertEquals(4.46, neoWheelMetersPerSec / driveReduction(13, 22), 0.01);
        assertEquals(4.80, neoWheelMetersPerSec / driveReduction(14, 22), 0.01);
        // As constantes usam uma combinação que encaixa no módulo: kit (12–14 com 22) ou
        // upgrade REV-21-3008 (14 com 21/20; 15 ou 16 com 20/19).
        int p = DRIVE_PINION_TEETH;
        int s = DRIVE_SPUR_TEETH;
        boolean kit = s == 22 && p >= 12 && p <= 14;
        boolean upgrade = (p == 14 && (s == 21 || s == 20)) || ((p == 15 || p == 16) && (s == 20 || s == 19));
        assertTrue(kit || upgrade, "pinhão " + p + " com engrenagem " + s + " não é combinação do kit REV-21-3005 nem do upgrade REV-21-3008");
        assertEquals(driveReduction(p, s), DRIVE_REDUCTION, 1e-12);
        assertEquals(neoWheelMetersPerSec / DRIVE_REDUCTION, DRIVE_FREE_SPEED_MPS, 1e-9);
        // Giro: UltraPlanetary 3:1 (84:29) e 4:1 (76:21) + 14 dentes na engrenagem de 62.
        assertEquals(9424.0 / 203.0, TURN_REDUCTION, 1e-9);
        assertEquals(46.42, TURN_REDUCTION, 0.01);
        assertEquals(12.0 / DRIVE_FREE_SPEED_MPS, DRIVE_KV, 1e-9);
        // Limites do template REV: 50 A no NEO e 20 A no NEO 550.
        assertEquals(50, DRIVE_CURRENT_LIMIT_AMPS);
        assertEquals(20, TURN_CURRENT_LIMIT_AMPS);
        assertEquals(edu.wpi.first.math.system.plant.DCMotor.getNEO(1).KvRadPerSecPerVolt,
            DRIVE_SIM_MOTOR.KvRadPerSecPerVolt, 1e-9);
        assertEquals(edu.wpi.first.math.system.plant.DCMotor.getNeo550(1).KvRadPerSecPerVolt,
            TURN_SIM_MOTOR.KvRadPerSecPerVolt, 1e-9);
    }

    @Test void sparkConfigurationsBuildWithTheRevLib2026() {
        assertNotNull(SwerveIOSparkMax.driveConfiguration(true));
        assertNotNull(SwerveIOSparkMax.driveConfiguration(false));
        assertNotNull(SwerveIOSparkMax.turnConfiguration());
    }

    @Test void absoluteEncodersUseValidDioPortsThatDoNotRepeat() {
        int[] dios = {FE_ABSOLUTE_ENCODER_DIO, FD_ABSOLUTE_ENCODER_DIO, TE_ABSOLUTE_ENCODER_DIO, TD_ABSOLUTE_ENCODER_DIO};
        double[] offsets = {FE_ABSOLUTE_OFFSET_DEG, FD_ABSOLUTE_OFFSET_DEG, TE_ABSOLUTE_OFFSET_DEG, TD_ABSOLUTE_OFFSET_DEG};
        var all = new HashSet<Integer>();
        for (int i = 0; i < 4; i++) {
            int dio = dios[i];
            if (dio == NO_ABSOLUTE_ENCODER) {
                // Sem absoluto o 0° só pode vir do zero ao ligar.
                assertEquals(0.0, offsets[i], "módulo sem encoder absoluto tem que ter offset 0.0 (zero ao ligar)");
                continue;
            }
            assertTrue(dio >= 0 && dio <= 9, "DIO " + dio + " fora de 0 a 9 (roboRIO)");
            assertTrue(all.add(dio), "DIO " + dio + " repetido");
        }
        for (double offset : offsets) {
            assertTrue(offset >= 0.0 && offset < 360.0, "offset " + offset + "° fora de 0 a 360");
        }
    }

    @Test void neo550TurnsBecomeModuleRadians() {
        // 46,42 voltas do NEO 550 = 1 volta do módulo.
        assertEquals(2.0 * Math.PI, TURN_MOTOR_POSITION_FACTOR * TURN_REDUCTION, 1e-12);
        assertEquals(TURN_MOTOR_POSITION_FACTOR / 60.0, TURN_MOTOR_VELOCITY_FACTOR, 1e-15);
    }

    @Test void absoluteAngleIsCounterClockwiseFromTheStraightWheelReading() {
        double tau = 2.0 * Math.PI;
        assertEquals(1.0, SwerveIOSparkMax.counterClockwise(1.0, false), 1e-12);
        assertEquals(tau - 1.0, SwerveIOSparkMax.counterClockwise(1.0, true), 1e-12);
        assertEquals(0.0, SwerveIOSparkMax.counterClockwise(0.0, true), 1e-12, "0° continua 0°, não 360°");
        assertEquals(0.0, SwerveIOSparkMax.counterClockwise(tau, false), 1e-12);
        // Calibração: leitura com a roda reta vira o 0° do módulo.
        double straight = SwerveIOSparkMax.counterClockwise(Math.toRadians(250.0), true);
        assertEquals(0.0, SwerveIOSparkMax.absoluteAngle(straight, straight), 1e-12);
        // Módulo girado 30° no sentido anti-horário: o sensor (invertido) desce 30°.
        double turned = SwerveIOSparkMax.counterClockwise(Math.toRadians(220.0), true);
        assertEquals(30.0, Math.toDegrees(SwerveIOSparkMax.absoluteAngle(turned, straight)), 1e-9);
        // Atravessa o 0°/360° sem salto: leitura 5° com offset 355° = +10°.
        assertEquals(10.0, Math.toDegrees(SwerveIOSparkMax.absoluteAngle(Math.toRadians(5.0),
            Math.toRadians(355.0))), 1e-9);
    }

    @Test void offsetLineIsReadyToPasteEvenOnAPortugueseComputer() {
        Locale previous = Locale.getDefault();
        try {
            Locale.setDefault(Locale.forLanguageTag("pt-BR"));
            assertEquals("public static final double FE_ABSOLUTE_OFFSET_DEG = 123.46;",
                SwerveIOSparkMax.offsetLine("FE", Math.toRadians(123.456)));
            assertEquals("public static final double TD_ABSOLUTE_OFFSET_DEG = 350.00;",
                SwerveIOSparkMax.offsetLine("TD", Math.toRadians(-10.0)));
        } finally {
            Locale.setDefault(previous);
        }
    }

    @Test void simulatedAbsoluteEncoderFollowsTheModule() {
        left = 0.5;
        cycles(50);
        for (int i = 0; i < 4; i++) {
            var in = swerve.getModule(i).getInputs();
            assertTrue(in.absoluteConnected);
            assertEquals(0.0, MathUtil.angleModulus(in.turnPositionRad - in.absoluteAngleRad), 1e-9);
        }
    }

    @Test void onlySparkFirmware26IsAccepted() {
        // Número da REVLib: maior (8 bits), menor (8 bits), build (16 bits).
        int v26 = (26 << 24) | (1 << 16) | 6;
        int v27 = 27 << 24;
        int v25 = (25 << 24) | (5 << 16) | 3;
        assertEquals("26.1.6", SwerveIOSparkMax.firmwareText(v26));
        assertEquals("desconhecido", SwerveIOSparkMax.firmwareText(0));
        assertTrue(SwerveIOSparkMax.firmwareProblem(v26) == null, "26.x é a da REVLib 2026");
        assertTrue(SwerveIOSparkMax.firmwareProblem(0) == null, "desconhecido: as leituras decidem");
        assertTrue(SwerveIOSparkMax.firmwareProblem(v27).contains("fatores de conversão"));
        assertTrue(SwerveIOSparkMax.firmwareProblem(v25).contains("antigo"));
        assertTrue(SwerveIOSparkMax.firmwareProblem(v27).contains(SPARK_FIRMWARE_RECOMMENDED));
    }

    // #endregion

    // #region PILOTAGEM ORIENTADA AO CAMPO

    @Test void leftStickForwardDrivesAwayFromTheBlueDriverStation() {
        fullSpeed();
        forward = 1.0;
        cycles(75);
        var pose = swerve.getPose();
        assertTrue(pose.getX() > 3.0, "andou " + pose.getX() + " m em 1,5 s");
        assertEquals(0.0, pose.getY(), 0.15, "foi reto");
        assertEquals(0.0, pose.getRotation().getDegrees(), 1.0, "não girou");
        assertTrue(speedMetersPerSec() > 0.9 * MAX_SPEED_MPS, "velocidade " + speedMetersPerSec());
        for (int i = 0; i < 4; i++) {
            assertWheelAngle(0.0, swerve.getModule(i).getAngle(), swerve.getModule(i).getName());
        }
    }

    @Test void leftStickLeftDrivesToFieldPlusY() {
        fullSpeed();
        left = 0.5;
        cycles(75);
        var pose = swerve.getPose();
        assertTrue(pose.getY() > 0.8, "andou " + pose.getY() + " m para a esquerda");
        assertEquals(0.0, pose.getX(), 0.1);
        for (int i = 0; i < 4; i++) {
            assertWheelAngle(90.0, swerve.getModule(i).getAngle(), swerve.getModule(i).getName());
        }
    }

    @Test void rightStickToTheLeftTurnsCounterClockwiseInPlace() {
        fullSpeed();
        counterClockwise = 1.0;
        cycles(50);
        assertTrue(gyroYawDegrees() > 200.0, "girou " + gyroYawDegrees() + "° em 1 s");
        assertEquals(0.0, swerve.getPose().getTranslation().getNorm(), 0.05, "girou no lugar");
        // Rodas tangentes ao círculo: 45° nos cantos (FD e TE a +45°, FE e TD a −45°).
        assertWheelAngle(-45.0, swerve.getModule(0).getAngle(), "FE");
        assertWheelAngle(45.0, swerve.getModule(1).getAngle(), "FD");
        assertWheelAngle(45.0, swerve.getModule(2).getAngle(), "TE");
        assertWheelAngle(-45.0, swerve.getModule(3).getAngle(), "TD");
    }

    @Test void forwardStaysForwardOnTheFieldAfterTheRobotTurns() {
        fullSpeed();
        swerve.resetPose(new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(90.0)));
        cycle();
        forward = 0.5;
        cycles(75);
        var pose = swerve.getPose();
        assertTrue(pose.getX() > 1.8, "andou para +X do campo: " + pose);
        assertEquals(1.0, pose.getY(), 0.15, "sem andar de lado no campo");
        assertEquals(90.0, pose.getRotation().getDegrees(), 1.0, "a frente do robô continua a 90°");
        // Robô virado para a esquerda: andar para +X do campo é andar para a direita dele.
        assertWheelAngle(-90.0, swerve.getModule(0).getAngle(), "FE");
    }

    @Test void onTheRedAllianceForwardIsTowardFieldMinusX() {
        fullSpeed();
        setAlliance(AllianceStationID.Red1);
        swerve.zeroHeading();
        cycle();
        assertEquals(180.0, Math.abs(swerve.getHeading().getDegrees()), 0.5, "frente para o lado azul");
        forward = 0.5;
        cycles(75);
        assertTrue(swerve.getPose().getX() < -0.8, "afastou da Driver Station vermelha: " + swerve.getPose());
        assertEquals(0.0, swerve.getPose().getY(), 0.15);
    }

    @Test void onRedTheRobotStartsFacingAwayFromItsDriverStation() {
        // Simulação de 19/11/2026 (Red 1): sem zerar a frente, o robô começava de frente para o
        // piloto e, orientado ao robô, "para a frente" vinha na direção dele.
        DriverStationSim.setEnabled(false);
        setAlliance(AllianceStationID.Red1);
        create(new SwerveGyroIOSim());
        fullSpeed(); // o swerve foi recriado: o 100% vale de novo para este
        cycles(2);
        assertEquals(180.0, Math.abs(swerve.getHeading().getDegrees()), 0.5, "frente para longe da DS vermelha");
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        swerve.toggleFieldCentric();
        forward = 0.5;
        cycles(75);
        assertTrue(swerve.getPose().getX() < -0.8, "orientado ao robô, foi para longe da DS vermelha: " + swerve.getPose());
        assertEquals(180.0, Math.abs(swerve.getHeading().getDegrees()), 1.0, "habilitado, a frente não muda sozinha");
    }

    @Test void zeroHeadingMakesTheCurrentFrontTheFieldForward() {
        fullSpeed();
        counterClockwise = 0.5;
        cycles(30);
        counterClockwise = 0.0;
        cycles(40);
        assertNotEquals(0.0, swerve.getHeading().getDegrees(), 1.0);
        swerve.zeroHeading();
        cycle();
        assertEquals(0.0, swerve.getHeading().getDegrees(), 0.5);
        double x0 = swerve.getPose().getX();
        forward = 0.5;
        cycles(75);
        assertTrue(swerve.getPose().getX() - x0 > 0.8, "agora a frente do robô é a frente do campo");
    }

    @Test void zeroDriverHeadingChangesOnlyTheDriverReference() {
        swerve.resetPose(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0)));
        swerve.zeroDriverHeading();
        // A odometria não muda...
        assertEquals(1.0, swerve.getPose().getX(), 1e-9);
        assertEquals(2.0, swerve.getPose().getY(), 1e-9);
        assertEquals(90.0, swerve.getHeading().getDegrees(), 1e-6);
        // ...só a frente que o piloto enxerga (aliança azul: 0°).
        assertEquals(0.0, swerve.getDriverHeading().getDegrees(), 1e-6);
    }

    @Test void yTogglesToRobotCentricAndForwardFollowsTheRobotFront() {
        fullSpeed();
        assertTrue(swerve.isFieldCentric(), "começa orientado ao campo");
        swerve.resetPose(new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(90.0)));
        swerve.toggleFieldCentric();
        assertFalse(swerve.isFieldCentric());
        cycle();
        forward = 0.5;
        cycles(75);
        var pose = swerve.getPose();
        // Robô virado para +Y do campo: para a frente do robô = +Y do campo (no modo campo seria +X).
        assertTrue(pose.getY() > 1.8, "andou para a frente do robô: " + pose);
        assertEquals(1.0, pose.getX(), 0.15, "sem andar para +X do campo");
        for (int i = 0; i < 4; i++) {
            assertWheelAngle(0.0, swerve.getModule(i).getAngle(), swerve.getModule(i).getName());
        }
        swerve.toggleFieldCentric();
        assertTrue(swerve.isFieldCentric(), "Y de novo volta para o campo");
    }

    @Test void startAlignsEveryWheelToZeroWithoutFlipping180() {
        scheduler.cancelAll();
        swerve.close();
        // Rodas tortas, algumas perto de 180° (a otimização deixaria essas invertidas).
        modules = new SwerveIOSim[] {new SwerveIOSim(Math.toRadians(170.0)), new SwerveIOSim(Math.toRadians(-120.0)),
            new SwerveIOSim(Math.toRadians(100.0)), new SwerveIOSim(Math.toRadians(-179.0))};
        swerve = new Swerve(new SwerveGyroIOSim(), modules[0], modules[1], modules[2], modules[3]);
        scheduler.schedule(Commands.run(swerve::alignWheelsStraight, swerve));
        cycles(75);
        assertEquals("ALINHANDO_RODAS_EM_0", swerve.getState());
        for (int i = 0; i < 4; i++) {
            var module = swerve.getModule(i);
            // O Rotation2d não dá a volta sozinho (359,4° = −0,6°): compara de −180° a 180°.
            double degrees = MathUtil.inputModulus(module.getAngle().getDegrees(), -180.0, 180.0);
            assertEquals(0.0, degrees, 2.0, module.getName() + " em 0°, não em 180°");
            assertEquals(0.0, module.getInputs().driveVelocityMetersPerSec, 0.05, module.getName() + " parada");
        }
        assertEquals(0.0, swerve.getPose().getTranslation().getNorm(), 0.05, "o robô não andou");
    }

    @Test void deadbandIgnoresAWornStick() {
        forward = 0.9 * JOYSTICK_DEADBAND;
        left = -0.9 * JOYSTICK_DEADBAND;
        counterClockwise = 0.9 * JOYSTICK_DEADBAND;
        cycles(50);
        assertEquals(0.0, speedMetersPerSec(), 1e-6);
        assertEquals(0.0, swerve.getPose().getTranslation().getNorm(), 1e-6);
    }

    @Test void holdingLtDrivesSlowly() {
        slow = true;
        forward = 1.0;
        cycles(100);
        // O LT continua existindo no comando; agora vale em cima do padrão de 50%.
        assertEquals(SPEED_DEFAULT * TELEOP_SLOW_SCALE * MAX_SPEED_MPS, speedMetersPerSec(), 0.15);
    }

    @Test void speedStartsAtFiftyPercent() {
        assertEquals(0.5, SPEED_DEFAULT, 1e-12);
        assertEquals(SPEED_DEFAULT, swerve.getDriverSpeedScale(), 1e-12);
        assertFalse(swerve.isReducedSpeed());
        forward = 1.0;
        cycles(100);
        assertEquals(SPEED_DEFAULT * MAX_SPEED_MPS, speedMetersPerSec(), 0.15);
    }

    @Test void bTogglesEightyPercentUntilPressedAgain() {
        forward = 1.0;
        swerve.toggleSpeedScale(SPEED_FAST);
        assertEquals(SPEED_FAST, swerve.getDriverSpeedScale(), 1e-12);
        assertFalse(swerve.isReducedSpeed());
        cycles(100);
        assertEquals(SPEED_FAST * MAX_SPEED_MPS, speedMetersPerSec(), 0.15);
        // Outro toque no B: volta a 50%.
        swerve.toggleSpeedScale(SPEED_FAST);
        assertEquals(SPEED_DEFAULT, swerve.getDriverSpeedScale(), 1e-12);
        cycles(100);
        assertEquals(SPEED_DEFAULT * MAX_SPEED_MPS, speedMetersPerSec(), 0.15);
    }

    @Test void xTogglesThirtyPercentUntilPressedAgain() {
        forward = 1.0;
        swerve.toggleSpeedScale(SPEED_SLOW);
        assertEquals(SPEED_SLOW, swerve.getDriverSpeedScale(), 1e-12);
        assertTrue(swerve.isReducedSpeed());
        cycles(100);
        assertEquals(SPEED_SLOW * MAX_SPEED_MPS, speedMetersPerSec(), 0.15);
        // Com o LT também: as duas reduções juntas.
        slow = true;
        cycles(100);
        assertEquals(SPEED_SLOW * TELEOP_SLOW_SCALE * MAX_SPEED_MPS, speedMetersPerSec(), 0.15);
        // Outro toque no X: volta a 50%.
        slow = false;
        swerve.toggleSpeedScale(SPEED_SLOW);
        assertFalse(swerve.isReducedSpeed());
        cycles(100);
        assertEquals(SPEED_DEFAULT * MAX_SPEED_MPS, speedMetersPerSec(), 0.15);
    }

    @Test void pressingTheOtherSpeedButtonSwitchesDirectly() {
        swerve.toggleSpeedScale(SPEED_SLOW);
        swerve.toggleSpeedScale(SPEED_FAST); // X e depois B: vai direto para 80%
        assertEquals(SPEED_FAST, swerve.getDriverSpeedScale(), 1e-12);
        swerve.toggleSpeedScale(SPEED_FAST);
        assertEquals(SPEED_DEFAULT, swerve.getDriverSpeedScale(), 1e-12);
    }

    @Test void accelerationIsLimitedForTheGearsAndTheTallPivot() {
        forward = 1.0;
        cycle();
        cycle();
        // Dois ciclos: no máximo 2 × 0,02 s × 8 m/s² = 0,32 m/s pedidos.
        assertTrue(speedMetersPerSec() <= 2 * 0.02 * TELEOP_MAX_ACCELERATION_MPS2 + 0.05,
            "velocidade " + speedMetersPerSec());
    }

    @Test void inAutonomousTheDriveCommandHoldsTheRobotStill() {
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
        forward = 1.0;
        counterClockwise = 1.0;
        cycles(50);
        assertEquals(0.0, swerve.getPose().getTranslation().getNorm(), 0.01);
        assertEquals(0.0, gyroYawDegrees(), 0.5);
    }

    // #endregion

    // #region CONFERÊNCIA DO GIRO (NEO 550 × ENCODER ABSOLUTO)

    /**
     * Gira o módulo de 10° em 10° até `toDeg` com o NEO 550 atrasado um ciclo (status do
     * Spark) e para. state = {posição do módulo, leitura do NEO 550} em graus. O absoluto
     * lê `absoluteGain` × posição; motorMoves = false simula o absoluto de outro módulo
     * (só ele anda, o NEO 550 deste fica parado).
     */
    private static void turnModule(SwerveConferenciaGiro check, double[] state, double toDeg,
            double absoluteGain, boolean motorMoves) {
        while (Math.abs(toDeg - state[0]) > 1e-9) {
            double step = Math.max(-10.0, Math.min(10.0, toDeg - state[0]));
            if (motorMoves) state[1] = state[0];
            state[0] += step;
            check.update(Math.toRadians(state[1]), absolute(state[0], absoluteGain), true, false);
        }
        if (motorMoves) state[1] = state[0];
        for (int i = 0; i < 4; i++) {
            check.update(Math.toRadians(state[1]), absolute(state[0], absoluteGain), true, i >= 2);
        }
    }

    private static double absolute(double positionDeg, double gain) {
        return SwerveIOSparkMax.wrapPositive(Math.toRadians(gain * positionDeg));
    }

    @Test void turnCheckAcceptsTheEncodersMovingTogetherEvenWithTheSparkDelay() {
        var check = new SwerveConferenciaGiro();
        double[] position = {350.0, 350.0};
        check.update(Math.toRadians(350.0), absolute(350.0, 1.0), true, true);
        turnModule(check, position, 440.0, 1.0, true); // atravessa o 0°/360°
        turnModule(check, position, 300.0, 1.0, true);
        turnModule(check, position, 390.0, 1.0, true);
        assertNull(check.problem());
        assertEquals(1.0, check.lastRatio(), 1e-9);
    }

    @Test void turnCheckWaitsForTheWholeWindowBeforeJudging() {
        double window = TURN_CHECK_WINDOW_DEGREES;
        assertEquals(45.0, window, 1e-9, "janela de 45° desde 19/11/2026 (era 15°)");
        var check = new SwerveConferenciaGiro();
        double[] position = {0.0, 0.0};
        check.update(0.0, absolute(0.0, -1.0), true, true);
        turnModule(check, position, 0.6 * window, -1.0, true);
        assertTrue(Double.isNaN(check.lastRatio()), "menos que a janela ainda não conta");
        turnModule(check, position, 1.2 * window, -1.0, true);
        assertEquals(-1.0, check.lastRatio(), 1e-9);
        assertNull(check.problem(), "uma janela errada ainda não para o módulo");
    }

    @Test void turnCheckStopsTheModuleWhenTheAbsoluteTurnsTheOtherWay() {
        var check = new SwerveConferenciaGiro();
        double[] position = {0.0, 0.0};
        check.update(0.0, absolute(0.0, -1.0), true, true);
        turnModule(check, position, 90.0, -1.0, true);
        assertNull(check.problem());
        turnModule(check, position, 0.0, -1.0, true);
        assertEquals(SwerveConferenciaGiro.INVERTIDO, check.problem());
        // Corrigido (ex.: TURN_ENCODER_INVERTED) e realinhado: 3 janelas certas liberam.
        check.restartWindow();
        check.update(0.0, absolute(0.0, 1.0), true, true);
        turnModule(check, position, 50.0, 1.0, true);
        turnModule(check, position, 100.0, 1.0, true);
        assertEquals(SwerveConferenciaGiro.INVERTIDO, check.problem(), "2 certas ainda não bastam");
        turnModule(check, position, 150.0, 1.0, true);
        assertNull(check.problem());
    }

    @Test void turnCheckCatchesAStuckAbsoluteAndAnAbsoluteFromAnotherModule() {
        var stuck = new SwerveConferenciaGiro();
        double[] position = {0.0, 0.0};
        stuck.update(0.0, absolute(0.0, 0.0), true, true);
        turnModule(stuck, position, 60.0, 0.0, true);
        turnModule(stuck, position, 0.0, 0.0, true);
        assertEquals(SwerveConferenciaGiro.ABSOLUTO_PARADO, stuck.problem());

        var other = new SwerveConferenciaGiro();
        position[0] = 0.0;
        position[1] = 0.0;
        other.update(0.0, absolute(0.0, 1.0), true, true);
        turnModule(other, position, 60.0, 1.0, false);
        turnModule(other, position, 0.0, 1.0, false);
        assertEquals(SwerveConferenciaGiro.ABSOLUTO_DE_OUTRO, other.problem());

        // Redução errada (absoluto anda metade do que o NEO 550 diz).
        var ratio = new SwerveConferenciaGiro();
        position[0] = 0.0;
        position[1] = 0.0;
        ratio.update(0.0, absolute(0.0, 0.5), true, true);
        turnModule(ratio, position, 90.0, 0.5, true);
        turnModule(ratio, position, 0.0, 0.5, true);
        assertEquals(SwerveConferenciaGiro.NAO_CONFERE, ratio.warning(), "proporção fora: aviso");
        assertNull(ratio.problem(), "proporção fora não para o módulo");
    }

    @Test void aStrangeRatioInTheSameDirectionOnlyWarnsAndTheModuleKeepsDriving() {
        // TE em 19/11/2026: "PARADO: GIRO_NAO_CONFERE (DIO 2, absoluto/NEO 550 = 1.54)".
        var check = new SwerveConferenciaGiro();
        double[] position = {0.0, 0.0};
        check.update(0.0, absolute(0.0, 1.54), true, true);
        turnModule(check, position, 60.0, 1.54, true);
        turnModule(check, position, 0.0, 1.54, true);
        assertNull(check.problem(), "a TE não para mais");
        assertEquals(SwerveConferenciaGiro.NAO_CONFERE, check.warning());
        assertEquals(1.54, check.lastRatio(), 0.01);
        // 3 janelas certas apagam o aviso.
        for (double to : new double[] {60.0, 0.0, 60.0}) turnModule(check, position, to, 1.0, true);
        assertNull(check.warning());
        assertNull(check.problem());
    }

    @Test void turnCheckDoesNotStopTheModuleWhenTheAbsoluteClosesTheWindowFirst() {
        // Robô em 19/11/2026: FE e TE paravam com "ENCODER_ABSOLUTO_DE_OUTRO_MODULO" e razão
        // 1,06 a 1,09. Com a folga, o absoluto fecha a janela antes do NEO 550 (1° antes).
        var check = new SwerveConferenciaGiro();
        double[] position = {0.0, 0.0};
        check.update(0.0, absolute(0.0, 1.065), true, true);
        double turn = TURN_CHECK_WINDOW_DEGREES - 1.0;
        double target = 0.0;
        for (int i = 0; i < 6; i++) {
            target += i % 2 == 0 ? turn : -turn;
            turnModule(check, position, target, 1.065, true);
        }
        assertNull(check.problem());
        assertEquals(1.065, check.lastRatio(), 0.01);
    }

    @Test void repeatedAbsoluteSignalIsCaughtOnlyWhileTurning() {
        // DIO 3 (TD) lendo o encoder da FE (DIO 0): 156,04° × 156,06° ao ligar, 19/11/2026.
        var detector = new SwerveSinalRepetido(4);
        double[] raw = {Math.toRadians(156.06), Math.toRadians(51.96), Math.toRadians(27.69), Math.toRadians(156.04)};
        for (int i = 0; i < 200; i++) assertEquals(-1, detector.update(raw), "parados e iguais por acaso: não acusa");
        int found = -1;
        for (int i = 0; i < 30 && found < 0; i++) {
            raw[0] += Math.toRadians(2.0);
            raw[3] = raw[0];
            raw[1] += Math.toRadians(2.0); // FD gira junto, mas com outra leitura
            found = detector.update(raw);
        }
        assertEquals(3, found, "par FE (0) × TD (3)");
        assertTrue(detector.isRepeated(3, 0));
        assertFalse(detector.isRepeated(0, 1));
        // Sem sinal (NaN) não conta.
        var noSignal = new SwerveSinalRepetido(4);
        double[] nan = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};
        for (int i = 0; i < 100; i++) assertEquals(-1, noSignal.update(nan));
    }

    @Test void turnCheckIgnoresCyclesWithoutTheAbsoluteSignal() {
        var check = new SwerveConferenciaGiro();
        double[] position = {0.0, 0.0};
        check.update(0.0, absolute(0.0, -1.0), true, true);
        turnModule(check, position, 90.0, -1.0, true);
        check.update(Math.toRadians(90.0), Double.NaN, false, true); // cabo solto: janela recomeça
        turnModule(check, position, 0.0, -1.0, true);
        assertNull(check.problem(), "sem janela completa depois da falha de sinal");
    }

    // #endregion

    // #region RODAS EM X

    @Test void setXPointsTheWheelsInAnX() {
        var lock = Commands.run(swerve::setX, swerve);
        scheduler.schedule(lock);
        cycles(25);
        assertEquals("TRAVADO_EM_X", swerve.getState());
        assertWheelAngle(45.0, swerve.getModule(0).getAngle(), "FE");
        assertWheelAngle(-45.0, swerve.getModule(1).getAngle(), "FD");
        assertWheelAngle(-45.0, swerve.getModule(2).getAngle(), "TE");
        assertWheelAngle(45.0, swerve.getModule(3).getAngle(), "TD");
        assertEquals(0.0, speedMetersPerSec(), 0.01);
    }

    // #endregion

    // #region SEGURANÇA

    @Test void currentLimitsHoldDuringAFullAcceleration() {
        forward = 1.0;
        counterClockwise = 1.0;
        double driveMax = 0.0;
        double turnMax = 0.0;
        for (int i = 0; i < 100; i++) {
            cycle();
            for (int m = 0; m < 4; m++) {
                driveMax = Math.max(driveMax, swerve.getModule(m).getInputs().driveCurrentAmps);
                turnMax = Math.max(turnMax, swerve.getModule(m).getInputs().turnCurrentAmps);
            }
        }
        assertTrue(driveMax <= DRIVE_CURRENT_LIMIT_AMPS + 0.5, "tração " + driveMax + " A");
        assertTrue(turnMax <= TURN_CURRENT_LIMIT_AMPS + 0.5, "giro " + turnMax + " A");
    }

    @Test void disablingStopsTheMotorsAndBrakes() {
        fullSpeed();
        forward = 1.0;
        cycles(75);
        assertTrue(speedMetersPerSec() > 3.0);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        cycles(75);
        assertEquals("DESABILITADO", swerve.getState());
        assertEquals(0.0, speedMetersPerSec(), 0.05, "Brake segurou o robô");
        for (int i = 0; i < 4; i++) {
            assertEquals(0.0, swerve.getModule(i).getInputs().driveAppliedVolts, 1e-9);
            assertEquals(0.0, swerve.getModule(i).getInputs().turnAppliedVolts, 1e-9);
        }
    }

    @Test void withoutAnyRequestTheMotorsStopAfterTheWatchdog() {
        scheduler.cancelAll();
        swerve.removeDefaultCommand();
        swerve.runVelocity(new ChassisSpeeds(2.0, 0.0, 0.0));
        cycles(5);
        assertEquals("DIRIGINDO", swerve.getState());
        cycles(20);
        assertEquals("SEM_PEDIDO_PARADO", swerve.getState());
        cycles(50);
        assertEquals(0.0, speedMetersPerSec(), 0.05);
    }

    @Test void withoutTheGyroTheHeadingComesFromTheWheels() {
        create(SwerveGyroIO.desligado());
        fullSpeed(); // o swerve foi recriado: o 100% vale de novo para este
        counterClockwise = 0.5;
        cycles(50);
        assertFalse(swerve.isGyroConnected());
        assertTrue(Math.abs(swerve.getHeading().getDegrees()) > 30.0,
            "a cinemática viu o robô girar: " + swerve.getHeading());
    }

    @Test void aModuleOffTheCanBusIsReportedAndTheOthersKeepWorking() {
        swerve.close();
        modules = new SwerveIOSim[] {null, new SwerveIOSim(), new SwerveIOSim(), new SwerveIOSim()};
        swerve = new Swerve(new SwerveGyroIOSim(), SwerveIO.desligado(), modules[1], modules[2], modules[3]);
        swerve.setDefaultCommand(new SwerveDirigirCommand(swerve, () -> forward, () -> left,
            () -> counterClockwise, () -> slow));
        fullSpeed(); // o swerve foi recriado: o 100% vale de novo para este
        forward = 0.5;
        cycles(50);
        assertTrue(swerve.getFault().contains("FE: PARADO: DESLIGADO_NAS_CONSTANTES"), swerve.getFault());
        assertTrue(swerve.getModule(1).getInputs().driveVelocityMetersPerSec > 0.8, "FD continua andando");
    }

    // #endregion

    // #endregion
}