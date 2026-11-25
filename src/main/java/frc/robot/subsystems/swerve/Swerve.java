package frc.robot.subsystems.swerve;

// #region 📚 IMPORTS

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// #endregion

/**
 * Swerve com 4 MAXSwerve de 3" (NEO na tração, NEO 550 no giro) e Pigeon 2.
 * Mesma lógica do DriveSubsystem do template REV v2026.0, com as boas práticas da
 * WPILib: discretização das velocidades, otimização + cosseno nos módulos, pose
 * estimada (pronta para visão) e campo visto da Driver Station de cada aliança.
 *
 * Usa os Spark MAX, os encoders absolutos (DIO do roboRIO) e o Pigeon reais no robô e
 * modelos físicos na simulação.
 */
public class Swerve extends SubsystemBase implements AutoCloseable {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    // #region 📦 IO E ESTADO

    private static final double LOOP_SECONDS = 0.02;
    // DIO do encoder absoluto de cada módulo, na ordem FE, FD, TE, TD.
    private static final int[] ABSOLUTE_DIOS = {FE_ABSOLUTE_ENCODER_DIO, FD_ABSOLUTE_ENCODER_DIO,
        TE_ABSOLUTE_ENCODER_DIO, TD_ABSOLUTE_ENCODER_DIO};

    private final SwerveModulo[] modules; // FE, FD, TE, TD
    private final SwerveGyroIO gyroIO;
    private final SwerveGyroIO.SwerveGyroIOInputs gyro = new SwerveGyroIO.SwerveGyroIOInputs();
    private final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(FE_LOCATION, FD_LOCATION, TE_LOCATION, TD_LOCATION);
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveVisualizer visualizer = new SwerveVisualizer();

    private SwerveModulePosition[] lastPositions;
    // Yaw usado pela odometria: o do Pigeon quando ele responde; sem ele, soma o giro
    // calculado pelas rodas. Ao voltar, o Pigeon continua de onde parou (sem salto).
    private double rawYawRad;
    private double gyroOffsetRad;
    private boolean gyroWasConnected;

    // 🛑 Dois DIOs lendo o mesmo encoder absoluto (ex.: DIO 3 com o sinal da FE, 19/11/2026).
    private final SwerveSinalRepetido repeatedSignal = new SwerveSinalRepetido(4);
    private final double[] absoluteRawRad = new double[4];
    private String repeatedSignalText = "NENHUM";

    // Último status de cada módulo avisado na Driver Station (só avisa quando muda).
    private final String[] reportedModuleStatus = new String[4];
    private final double[] reportedAtSeconds = {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
        Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
    private boolean wasEnabled;

    private boolean driving;
    private double lastRequestSeconds = Double.NEGATIVE_INFINITY;
    // Pilotagem: true = orientada ao campo (frente do analógico = longe da Driver Station);
    // false = orientada ao robô (frente do analógico = frente do robô). Botão Y alterna.
    private boolean fieldCentric = true;
    // Fração da velocidade do piloto no andar e no giro: 50% por padrão;
    // B alterna 80%, X alterna 30% (apertar de novo volta a 50%).
    private double speedScale = SPEED_DEFAULT;
    // Frente do piloto (Start + A): desvio entre o heading da odometria e a frente que o
    // piloto considera "para a frente". Só muda a pilotagem; a pose não é tocada.
    private Rotation2d driverOffset = Rotation2d.kZero;
    // Frente inicial pela aliança: enquanto ninguém zerou a frente (Start + A ou
    // zeroHeading) nem colocou uma pose, o robô desabilitado é considerado de frente
    // para longe da sua Driver Station.
    private boolean headingSetByTeam;
    private Alliance headingAlliance;
    private String state = "PARADO";
    private String fault = "NENHUMA";

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region ⚙️ CONSTRUÇÃO

    /**
     * Seleciona os Spark MAX e o Pigeon reais ou os modelos simulados. No robô, com
     * SWERVE_MOTORS_ENABLED / GYRO_ENABLED = false, a parte não é criada (desligada).
     */
    public Swerve() {
        this(!RobotBase.isReal() ? new SwerveGyroIOSim()
                : GYRO_ENABLED ? new SwerveGyroIOPigeon2() : SwerveGyroIO.desligado(),
            moduleIO("FE", FE_DRIVE_ID, FE_TURN_ID, FE_ABSOLUTE_ENCODER_DIO, FE_ABSOLUTE_OFFSET_DEG, FE_DRIVE_INVERTED),
            moduleIO("FD", FD_DRIVE_ID, FD_TURN_ID, FD_ABSOLUTE_ENCODER_DIO, FD_ABSOLUTE_OFFSET_DEG, FD_DRIVE_INVERTED),
            moduleIO("TE", TE_DRIVE_ID, TE_TURN_ID, TE_ABSOLUTE_ENCODER_DIO, TE_ABSOLUTE_OFFSET_DEG, TE_DRIVE_INVERTED),
            moduleIO("TD", TD_DRIVE_ID, TD_TURN_ID, TD_ABSOLUTE_ENCODER_DIO, TD_ABSOLUTE_OFFSET_DEG, TD_DRIVE_INVERTED));
    }

    /** Recebe os IOs prontos (testes): giroscópio e módulos na ordem FE, FD, TE, TD. */
    public Swerve(SwerveGyroIO gyroIO, SwerveIO fe, SwerveIO fd, SwerveIO te, SwerveIO td) {
        this.gyroIO = gyroIO;
        modules = new SwerveModulo[] {
            new SwerveModulo("FE", fe),
            new SwerveModulo("FD", fd),
            new SwerveModulo("TE", te),
            new SwerveModulo("TD", td)};
        lastPositions = getModulePositions();
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero, lastPositions, Pose2d.kZero);
    }

    private static SwerveIO moduleIO(String name, int driveId, int turnId, int absoluteDio,
            double absoluteOffsetDeg, boolean driveInverted) {
        if (!RobotBase.isReal()) return new SwerveIOSim();
        return SWERVE_MOTORS_ENABLED
            ? new SwerveIOSparkMax(name, driveId, turnId, absoluteDio, Math.toRadians(absoluteOffsetDeg), driveInverted)
            : SwerveIO.desligado();
    }

    // #endregion

    // #region 🛠️ CONSULTAS

    /** Pose estimada no campo (origem sempre no canto da aliança azul, como na WPILib). */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Para onde a frente do robô aponta no campo (anti-horário positivo). */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public SwerveModuleState[] getModuleStates() {
        var states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) states[i] = modules[i].getState();
        return states;
    }

    public SwerveModuleState[] getDesiredStates() {
        var states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) states[i] = modules[i].getDesiredState();
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        var positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) positions[i] = modules[i].getPosition();
        return positions;
    }

    /** Velocidades medidas pelas rodas, no referencial do robô. */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public boolean isGyroConnected() {
        return gyro.connected;
    }

    /** Módulo pela ordem FE, FD, TE, TD (telemetria e testes). */
    public SwerveModulo getModule(int index) {
        return modules[index];
    }

    public String getState() {
        return state;
    }

    public String getFault() {
        return fault;
    }

    /** true = pilotagem orientada ao campo; false = orientada ao robô. */
    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public void setFieldCentric(boolean fieldCentric) {
        if (this.fieldCentric == fieldCentric) return;
        this.fieldCentric = fieldCentric;
        DriverStation.reportWarning("Swerve: pilotagem " + (fieldCentric
            ? "ORIENTADA AO CAMPO (frente do analógico = longe da Driver Station)"
            : "ORIENTADA AO ROBÔ (frente do analógico = frente do robô)"), false);
    }

    /** Botão Y: campo ↔ robô. */
    public void toggleFieldCentric() {
        setFieldCentric(!fieldCentric);
    }

    /** true = abaixo do padrão (30%). */
    public boolean isReducedSpeed() {
        return speedScale < SPEED_DEFAULT;
    }

    /** B (80%) e X (30%): apertar o mesmo botão de novo volta ao padrão de 50%. */
    public void toggleSpeedScale(double target) {
        speedScale = (speedScale == target) ? SPEED_DEFAULT : target;
        DriverStation.reportWarning("Swerve: velocidade " + speedModeText(), false);
    }

    /** Fração da velocidade do piloto, no andar e no giro. */
    public double getDriverSpeedScale() {
        return speedScale;
    }

    private String speedModeText() {
        return Math.round(speedScale * 100.0) + "%";
    }

    /** Aliança vermelha na Driver Station (sem aliança informada: azul). */
    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    // #endregion

    // #region 🎯 CONTROLE - MOVIMENTO

    /**
     * Velocidades no referencial do robô (X frente, Y esquerda, anti-horário positivo).
     * Discretiza (o robô não "escorrega" de lado ao andar girando), converte para os
     * quatro módulos e limita a roda mais rápida a MAX_SPEED_MPS, mantendo a direção.
     */
    public void runVelocity(ChassisSpeeds robotRelativeSpeeds) {
        if (DriverStation.isDisabled() || !Double.isFinite(robotRelativeSpeeds.vxMetersPerSecond)
                || !Double.isFinite(robotRelativeSpeeds.vyMetersPerSecond)
                || !Double.isFinite(robotRelativeSpeeds.omegaRadiansPerSecond)) {
            stop();
            return;
        }
        var states = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(robotRelativeSpeeds, LOOP_SECONDS));
        boolean moving = robotRelativeSpeeds.vxMetersPerSecond != 0.0 || robotRelativeSpeeds.vyMetersPerSecond != 0.0
            || robotRelativeSpeeds.omegaRadiansPerSecond != 0.0;
        applyStates(states, moving ? "DIRIGINDO" : "PARADO_SEGURANDO");
    }

    /** Velocidades no referencial do campo (origem azul), convertidas pela frente atual do robô. */
    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getHeading()));
    }

    /** Estados prontos para cada módulo (ex.: seguidor de trajetória), ordem FE, FD, TE, TD. */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        if (DriverStation.isDisabled() || desiredStates.length != modules.length) {
            stop();
            return;
        }
        var states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = new SwerveModuleState(desiredStates[i].speedMetersPerSecond, desiredStates[i].angle);
        }
        applyStates(states, "SEGUINDO_ESTADOS");
    }

    /** Rodas em X (template REV): o robô resiste a ser empurrado. Chamar a cada ciclo. */
    public void setX() {
        if (DriverStation.isDisabled()) {
            stop();
            return;
        }
        applyStates(new SwerveModuleState[] {
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0))}, "TRAVADO_EM_X");
    }

    /**
     * Rodas paradas e apontadas para a frente (0°), sem inverter nenhuma 180°. Serve para
     * conferir o alinhamento com uma barra e para deixar as rodas retas antes de reiniciar
     * o código (a TD zera ao ligar). Hoje sem botão no controle. Chamar a cada ciclo.
     */
    public void alignWheelsStraight() {
        if (DriverStation.isDisabled()) {
            stop();
            return;
        }
        for (var module : modules) module.setAngleWithoutOptimizing(Rotation2d.kZero);
        driving = true;
        lastRequestSeconds = Timer.getTimestamp();
        state = "ALINHANDO_RODAS_EM_0";
    }

    /** Neutro nos 8 motores (Brake): usado ao desabilitar e pelas proteções. */
    public void stop() {
        driving = false;
        for (var module : modules) module.stop();
    }

    private void applyStates(SwerveModuleState[] states, String newState) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED_MPS);
        for (int i = 0; i < modules.length; i++) modules[i].setDesiredState(states[i]);
        driving = true;
        lastRequestSeconds = Timer.getTimestamp();
        state = newState;
    }

    // #endregion

    // #region 🧭 POSE E FRENTE DO ROBÔ

    /** Coloca o robô nesta pose do campo (ex.: início do autônomo). Não mexe no Pigeon. */
    public void resetPose(Pose2d pose) {
        headingSetByTeam = true;
        driverOffset = Rotation2d.kZero;
        poseEstimator.resetPosition(Rotation2d.fromRadians(rawYawRad), lastPositions, pose);
    }

    /**
     * Ao ligar, o Pigeon começa em 0°, que no campo é "de frente para a aliança vermelha".
     * Na aliança vermelha isso deixava o robô de frente para o próprio piloto: orientado
     * ao campo ficava invertido sem zerar a frente, e orientado ao robô "para a frente" vinha
     * na direção do piloto (visto na simulação de 19/11/2026, que começa no Red 1).
     * Enquanto o robô está desabilitado e ninguém zerou a frente, assume a posição de
     * largada: frente para longe da Driver Station (0° no azul, 180° no vermelho).
     */
    private void applyStartingHeading() {
        if (headingSetByTeam || !DriverStation.isDisabled()) return;
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty() || alliance.get() == headingAlliance) return;
        headingAlliance = alliance.get();
        poseEstimator.resetPosition(Rotation2d.fromRadians(rawYawRad), lastPositions,
            new Pose2d(getPose().getTranslation(), headingAlliance == Alliance.Red ? Rotation2d.kPi : Rotation2d.kZero));
    }

    /**
     * A frente atual do robô passa a ser "para longe da Driver Station": 0° na
     * aliança azul e 180° na vermelha (origem do campo sempre no lado azul).
     * Mexe na pose (odometria). Sem botão no controle: SmartDashboard Swerve/ZerarFrente.
     */
    public void zeroHeading() {
        resetPose(new Pose2d(getPose().getTranslation(), isRedAlliance() ? Rotation2d.kPi : Rotation2d.kZero));
    }

    /**
     * Heading que a pilotagem orientada ao campo deve usar: o da odometria menos a frente
     * do piloto (Start + A). Sem ajuste do piloto, é igual a getHeading().
     */
    public Rotation2d getDriverHeading() {
        return getHeading().minus(driverOffset);
    }

    /**
     * Start + A: a frente atual do robô passa a ser "para a frente" para o piloto. A pose
     * (odometria) não muda; só o desvio usado na pilotagem orientada ao campo.
     */
    public void zeroDriverHeading() {
        // Sem isto, a frente de largada pela aliança (applyStartingHeading) desfaria o ajuste.
        headingSetByTeam = true;
        Rotation2d forward = isRedAlliance() ? Rotation2d.kPi : Rotation2d.kZero;
        driverOffset = getHeading().minus(forward);
    }

    /** Pose medida pela visão (AprilTags), com o horário da foto (Timer.getTimestamp()). */
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
    }

    // #endregion

    // #region 🧭 ENCODERS ABSOLUTOS

    /**
     * Leitura ATUAL do absoluto como linha pronta para SwerveConstants.java. Com as rodas
     * retas (alinhadas com uma barra, robô desabilitado), copiada lá, o 0° passa a vir do
     * absoluto. Mostra a leitura de agora (e não o offset em uso) para servir também para
     * recalibrar um módulo que já tem offset fixo.
     */
    private String offsetReport(int index) {
        var in = modules[index].getInputs();
        String name = modules[index].getName();
        if (ABSOLUTE_DIOS[index] < 0) return "// " + name + ": sem encoder absoluto (desligado em SwerveConstants)";
        if (!in.absoluteConnected) {
            return "// " + name + ": sem leitura do encoder absoluto no DIO " + ABSOLUTE_DIOS[index];
        }
        return SwerveIOSparkMax.offsetLine(name, in.absoluteRawRad) + " // DIO " + ABSOLUTE_DIOS[index];
    }

    /** Avisa (uma vez por par) quando dois DIOs leem o mesmo encoder absoluto girando. */
    private void checkRepeatedAbsoluteSignal() {
        // Na simulação os 4 absolutos são perfeitos e sem offset: andando reto, leem igual.
        if (!RobotBase.isReal()) return;
        for (int i = 0; i < modules.length; i++) {
            var in = modules[i].getInputs();
            absoluteRawRad[i] = in.absoluteConnected ? in.absoluteRawRad : Double.NaN;
        }
        int pair = repeatedSignal.update(absoluteRawRad);
        if (pair < 0) return;
        int a = pair / modules.length;
        int b = pair % modules.length;
        String text = String.format("DIO %d (%s) e DIO %d (%s)", ABSOLUTE_DIOS[a], modules[a].getName(),
            ABSOLUTE_DIOS[b], modules[b].getName());
        repeatedSignalText = repeatedSignalText.equals("NENHUM") ? text : repeatedSignalText + "; " + text;
        DriverStation.reportError("Swerve: " + text + " estão lendo o MESMO encoder absoluto (iguais enquanto"
            + " giram). Conferir os fios: cada DIO tem que ir no encoder do próprio módulo. Robô desabilitado,"
            + " girar um módulo de cada vez com a mão e ver qual Swerve/XX/EncoderAbsolutoGraus muda.", false);
    }

    // #endregion

    // #region 📡 1. SENSORES E ODOMETRIA

    private void readSensors() {
        for (var module : modules) module.updateInputs();
        checkRepeatedAbsoluteSignal();
    }

    /** Giro do chassi pelas rodas (cinemática); o Pigeon tem prioridade quando responde. */
    private void updateOdometry() {
        var positions = getModulePositions();
        var twist = kinematics.toTwist2d(lastPositions, positions);
        lastPositions = positions;
        gyroIO.simulateYawStep(twist.dtheta);
        gyroIO.updateInputs(gyro);
        if (gyro.connected) {
            if (!gyroWasConnected) gyroOffsetRad = rawYawRad - gyro.yawRad;
            rawYawRad = gyro.yawRad + gyroOffsetRad;
        } else {
            rawYawRad += twist.dtheta;
        }
        gyroWasConnected = gyro.connected;
        poseEstimator.update(Rotation2d.fromRadians(rawYawRad), positions);
    }

    // #endregion

    // #region 🛑 2. SEGURANÇA - DESABILITADO, COMANDO PARADO E MÓDULOS

    private void applySafety() {
        if (DriverStation.isDisabled()) {
            // Neutro uma vez ao desabilitar (o Spark já para sozinho sem o sinal do roboRIO):
            // ao habilitar de novo, nenhum motor volta com o último pedido.
            if (driving || !state.equals("DESABILITADO")) stop();
            state = "DESABILITADO";
        } else if (driving && Timer.getTimestamp() - lastRequestSeconds > MOTOR_SAFETY_SECONDS) {
            // 🛑 Nenhum comando pediu movimento: não deixa a última velocidade rodando.
            stop();
            state = "SEM_PEDIDO_PARADO";
        }
        var faults = new StringBuilder();
        boolean enabledNow = DriverStation.isEnabled();
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            var in = module.getInputs();
            String status = !module.isConnected() ? "PARADO: " + in.configurationStatus
                : !in.activeFaults.isEmpty() ? "aviso (o módulo continua andando): " + in.activeFaults
                : in.configurationStatus.startsWith("FALHA_COMANDO") ? in.configurationStatus : "OK";
            // Zerando, esperando o encoder absoluto ou alinhando o giro (~0,1 s): só no painel.
            boolean transitional = !module.isConnected() && (in.configurationStatus.startsWith("ZERANDO")
                || in.configurationStatus.startsWith("AGUARDANDO") || in.configurationStatus.startsWith("ALINHANDO"));
            if (!status.equals("OK")) {
                if (faults.length() > 0) faults.append("; ");
                faults.append(module.getName()).append(": ").append(status);
            }
            // 📢 Avisa na Driver Station quando o status muda (no máximo 1 vez por segundo
            // por módulo, para um status que oscila não lotar o console) e de novo ao habilitar.
            boolean enabledEdge = enabledNow && !wasEnabled && !status.equals("OK");
            boolean changed = !status.equals(reportedModuleStatus[i]);
            double now = Timer.getTimestamp();
            if (!transitional && (enabledEdge || (changed && now - reportedAtSeconds[i] >= 1.0))) {
                if (!status.equals("OK") || reportedModuleStatus[i] != null) {
                    // Módulo parado = erro (vermelho na Driver Station); o resto é aviso.
                    if (status.startsWith("PARADO")) {
                        DriverStation.reportError("Swerve " + module.getName() + ": " + status, false);
                    } else {
                        DriverStation.reportWarning("Swerve " + module.getName() + ": " + status, false);
                    }
                    reportedAtSeconds[i] = now;
                }
                reportedModuleStatus[i] = status;
            }
        }
        wasEnabled = enabledNow;
        fault = faults.length() == 0 ? "NENHUMA" : faults.toString();
    }

    // #endregion

    // #region 📊 3. TELEMETRIA

    /** Painel (SmartDashboard/Swerve) e poses para o AdvantageScope. */
    private void publishTelemetry() {
        var command = getCurrentCommand();
        SmartDashboard.putString("Swerve/Comando", command == null ? "Nenhum" : command.getName());
        SmartDashboard.putString("Swerve/Estado", state);
        SmartDashboard.putString("Swerve/Pilotagem", fieldCentric ? "CAMPO" : "ROBO");
        SmartDashboard.putString("Swerve/Velocidade", speedModeText());
        SmartDashboard.putString("Swerve/Falha", fault);
        SmartDashboard.putString("Swerve/EncodersAbsolutosRepetidos", repeatedSignalText);
        SmartDashboard.putBoolean("Swerve/Giroscopio/Conectado", gyro.connected);
        SmartDashboard.putString("Swerve/Giroscopio/Estado", gyro.status);
        SmartDashboard.putNumber("Swerve/Giroscopio/YawGraus", Math.toDegrees(gyro.yawRad));

        var pose = getPose();
        var speeds = getRobotRelativeSpeeds();
        SmartDashboard.putNumber("Swerve/PoseXMetros", pose.getX());
        SmartDashboard.putNumber("Swerve/PoseYMetros", pose.getY());
        SmartDashboard.putNumber("Swerve/FrenteGraus", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Swerve/FrenteDoPilotoGraus", getDriverHeading().getDegrees());
        SmartDashboard.putNumber("Swerve/VelocidadeMetrosPorSeg",
            Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
        SmartDashboard.putNumber("Swerve/GiroGrausPorSeg", Math.toDegrees(speeds.omegaRadiansPerSecond));

        var measured = getModuleStates();
        var desired = getDesiredStates();
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            var in = module.getInputs();
            String key = "Swerve/" + module.getName() + "/";
            SmartDashboard.putBoolean(key + "Conectado", module.isConnected());
            SmartDashboard.putString(key + "Configuracao", in.configurationStatus);
            SmartDashboard.putString(key + "FalhasAtivas", in.activeFaults);
            SmartDashboard.putString(key + "Tracao/Firmware", in.driveFirmware);
            SmartDashboard.putString(key + "Giro/Firmware", in.turnFirmware);
            SmartDashboard.putNumber(key + "AnguloGraus", measured[i].angle.getDegrees());
            SmartDashboard.putNumber(key + "AlvoAnguloGraus", desired[i].angle.getDegrees());
            SmartDashboard.putNumber(key + "VelocidadeMetrosPorSeg", measured[i].speedMetersPerSecond);
            SmartDashboard.putNumber(key + "AlvoVelocidadeMetrosPorSeg", desired[i].speedMetersPerSecond);
            // Encoder absoluto no roboRIO: leitura anti-horária sem offset (com a roda reta,
            // é o offset), sinal, frequência (~976 Hz no V1) e diferença para o NEO 550.
            SmartDashboard.putBoolean(key + "EncoderAbsolutoConectado", in.absoluteConnected);
            SmartDashboard.putNumber(key + "EncoderAbsolutoGraus", Math.toDegrees(in.absoluteRawRad));
            SmartDashboard.putNumber(key + "EncoderAbsolutoHz", in.absoluteFrequencyHz);
            SmartDashboard.putNumber(key + "DiferencaAbsolutoGraus", in.absoluteConnected
                ? Math.toDegrees(MathUtil.angleModulus(in.turnPositionRad - in.absoluteAngleRad)) : 0.0);
            SmartDashboard.putString(key + "OffsetParaCopiar", offsetReport(i));
            SmartDashboard.putNumber(key + "Giro/RazaoAbsolutoPorMotor", in.turnCheckRatio);
            // Voltas do NEO desde que ligou (não depende da redução configurada):
            // girar a roda 10 voltas com a mão e dividir a diferença por 10 = redução real.
            SmartDashboard.putNumber(key + "Tracao/VoltasDoMotor", in.drivePositionMeters / DRIVE_POSITION_FACTOR);
            SmartDashboard.putNumber(key + "Tracao/CorrenteAmps", in.driveCurrentAmps);
            SmartDashboard.putNumber(key + "Tracao/Tensao", in.driveAppliedVolts);
            SmartDashboard.putNumber(key + "Giro/CorrenteAmps", in.turnCurrentAmps);
            SmartDashboard.putNumber(key + "Giro/Tensao", in.turnAppliedVolts);
            SmartDashboard.putNumber(key + "ReiniciosTracao", module.getDriveResetCount());
        }
        visualizer.update(pose, measured, desired, speeds);
    }

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 🧹 ENCERRAMENTO (TESTES)

    @Override
    public void close() {
        stop();
        for (var module : modules) module.close();
        gyroIO.close();
        visualizer.close();
        CommandScheduler.getInstance().unregisterSubsystem(this);
    }

    // #endregion

    // #region 🔁 PERIODIC - ORDEM DE CADA CICLO (20 ms)

    // Depois do periodic, os comandos (pasta commands/swerve) chamam as funções de
    // CONTROLE da parte 2: runVelocity, driveFieldRelative, setX...
    @Override
    public void periodic() {
        readSensors();
        updateOdometry();
        applyStartingHeading();
        applySafety();
        publishTelemetry();
    }

    // #endregion

    // #endregion
}