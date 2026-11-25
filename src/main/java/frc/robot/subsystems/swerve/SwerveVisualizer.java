package frc.robot.subsystems.swerve;

// #region 📚 IMPORTS

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayDeque;

// #endregion

/**
 * 📊 Swerve no AdvantageScope e na janela da simulação (NetworkTables, em
 * SmartDashboard/Swerve). Tudo também vai para o .wpilog do DataLogManager.
 *
 * - Pose (Pose2d): campo 2D/3D do AdvantageScope.
 * - EstadosMedidos e EstadosAlvo (SwerveModuleState[], ordem FE, FD, TE, TD):
 *   aba Swerve do AdvantageScope.
 * - Rotacao (Rotation2d) e VelocidadesChassi (ChassisSpeeds, referencial do robô).
 * - Campo (Field2d): robô no campo da janela Robot Simulation, do Glass e do Elastic,
 *   com o objeto "Rastro" (caminho dos últimos ~15 s).
 * - Rastro (Pose2d[]): o mesmo caminho para o AdvantageScope (trajetória).
 * - Modulos (SwerveDrive): widget de swerve do Elastic (4 rodas e a frente do robô).
 */
public final class SwerveVisualizer implements AutoCloseable {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    private final StructPublisher<Pose2d> posePublisher;
    private final StructArrayPublisher<SwerveModuleState> measuredPublisher;
    private final StructArrayPublisher<SwerveModuleState> desiredPublisher;
    private final StructPublisher<Rotation2d> rotationPublisher;
    private final StructPublisher<ChassisSpeeds> speedsPublisher;
    private final StructArrayPublisher<Pose2d> trailPublisher;
    private final Field2d field = new Field2d();

    // Rastro: uma pose a cada TRAIL_EVERY_CYCLES (10 Hz) se andou mais de 2 cm, até ~15 s.
    private static final int TRAIL_EVERY_CYCLES = 5;
    private static final int TRAIL_MAX_POSES = 150;
    private static final double TRAIL_MIN_STEP_METERS = 0.02;
    private final ArrayDeque<Pose2d> trail = new ArrayDeque<>();
    private int trailCycle;

    // Últimos valores para o widget SwerveDrive do Elastic (lidos pelo SmartDashboard).
    private SwerveModuleState[] lastMeasured = {new SwerveModuleState(), new SwerveModuleState(),
        new SwerveModuleState(), new SwerveModuleState()};
    private Rotation2d lastRotation = Rotation2d.kZero;

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region ⚙️ CONSTRUÇÃO

    SwerveVisualizer() {
        var table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Swerve");
        posePublisher = table.getStructTopic("Pose", Pose2d.struct).publish();
        measuredPublisher = table.getStructArrayTopic("EstadosMedidos", SwerveModuleState.struct).publish();
        desiredPublisher = table.getStructArrayTopic("EstadosAlvo", SwerveModuleState.struct).publish();
        rotationPublisher = table.getStructTopic("Rotacao", Rotation2d.struct).publish();
        speedsPublisher = table.getStructTopic("VelocidadesChassi", ChassisSpeeds.struct).publish();
        trailPublisher = table.getStructArrayTopic("Rastro", Pose2d.struct).publish();
        SmartDashboard.putData("Swerve/Campo", field);
        SmartDashboard.putData("Swerve/Modulos", elasticSwerveWidget());
    }

    /**
     * Widget "SwerveDrive" do Elastic (formato da documentação do Elastic): ângulo (rad) e
     * velocidade (m/s) de cada roda, ordem FE, FD, TE, TD, e a frente do robô (rad).
     */
    private Sendable elasticSwerveWidget() {
        return new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");
                String[] names = {"Front Left", "Front Right", "Back Left", "Back Right"};
                for (int i = 0; i < names.length; i++) {
                    int index = i;
                    builder.addDoubleProperty(names[i] + " Angle", () -> lastMeasured[index].angle.getRadians(), null);
                    builder.addDoubleProperty(names[i] + " Velocity", () -> lastMeasured[index].speedMetersPerSecond, null);
                }
                builder.addDoubleProperty("Robot Angle", () -> lastRotation.getRadians(), null);
            }
        };
    }

    @Override
    public void close() {
        posePublisher.close();
        measuredPublisher.close();
        desiredPublisher.close();
        rotationPublisher.close();
        speedsPublisher.close();
        trailPublisher.close();
        field.close();
    }

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 📊 TELEMETRIA

    /** Publica pose, estados dos módulos (medidos e pedidos) e velocidades do chassi. */
    public void update(Pose2d pose, SwerveModuleState[] measured, SwerveModuleState[] desired,
            ChassisSpeeds robotRelativeSpeeds) {
        posePublisher.set(pose);
        measuredPublisher.set(measured);
        desiredPublisher.set(desired);
        rotationPublisher.set(pose.getRotation());
        speedsPublisher.set(robotRelativeSpeeds);
        field.setRobotPose(pose);
        if (measured.length == 4) lastMeasured = measured;
        lastRotation = pose.getRotation();
        updateTrail(pose);
    }

    /** Caminho percorrido (odometria) para o AdvantageScope e o Field2d. */
    private void updateTrail(Pose2d pose) {
        if (++trailCycle < TRAIL_EVERY_CYCLES) return;
        trailCycle = 0;
        var last = trail.peekLast();
        if (last != null && last.getTranslation().getDistance(pose.getTranslation()) < TRAIL_MIN_STEP_METERS) return;
        trail.addLast(pose);
        if (trail.size() > TRAIL_MAX_POSES) trail.removeFirst();
        var poses = trail.toArray(new Pose2d[0]);
        trailPublisher.set(poses);
        field.getObject("Rastro").setPoses(poses);
    }

    // #endregion

    // #endregion
}
