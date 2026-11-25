package frc.robot.subsystems.manipulador;

// #region 📚 IMPORTS

import static frc.robot.subsystems.manipulador.ManipuladorConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

// #endregion

/**
 * 📊 Desenho 3D do manipulador no AdvantageScope (modelo
 * config/advantagescope/mecanismos/Robot_Manipulador_9460).
 *
 * Seis peças, sempre nesta ordem (model_0 a model_5):
 * 0 corpo (gira no eixo longo), 1 braço direito, 2 braço esquerdo,
 * 3 rodas direitas, 4 rodas esquerdas, 5 objeto (esfera de 15 cm).
 *
 * Cada pose é (pivô no referencial do manipulador, rotação da junta), mas as
 * juntas estão encadeadas: o braço gira junto com o corpo, e as rodas junto
 * com o braço. Por isso cada pose parte da pose da peça "mãe".
 */
public final class ManipuladorVisualizer implements AutoCloseable {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    // #region 🧭 PONTOS DO CAD (REFERENCIAL DO MANIPULADOR)

    private static final Translation3d ROLL_AXIS = new Translation3d(0.0, 0.0, ROLL_AXIS_HEIGHT_METERS);

    // Do eixo da rotação até o pivô de cada braço, e do pivô até o eixo das rodas.
    private static final Translation3d RIGHT_PIVOT = new Translation3d(PIVOT_X_METERS, PIVOT_RIGHT_Y_METERS, 0.0);
    private static final Translation3d LEFT_PIVOT = new Translation3d(PIVOT_X_METERS, PIVOT_LEFT_Y_METERS, 0.0);
    private static final Translation3d RIGHT_AXLE = new Translation3d(
        AXLE_RIGHT_X_METERS - PIVOT_X_METERS, AXLE_RIGHT_Y_METERS - PIVOT_RIGHT_Y_METERS, 0.0);
    private static final Translation3d LEFT_AXLE = new Translation3d(
        AXLE_LEFT_X_METERS - PIVOT_X_METERS, AXLE_LEFT_Y_METERS - PIVOT_LEFT_Y_METERS, 0.0);

    // O objeto fora de vista (abaixo do chão) quando a garra não segura nada.
    private static final Pose3d HIDDEN_OBJECT = new Pose3d(0.0, 0.0, -10.0, new Rotation3d());
    private static final Pose2d BASE_POSE = new Pose2d();

    private final StructPublisher<Pose2d> basePublisher;
    private final StructArrayPublisher<Pose3d> componentPublisher;

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region ⚙️ CONSTRUÇÃO

    ManipuladorVisualizer() {
        var table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Manipulador");
        basePublisher = table.getStructTopic("BasePose", Pose2d.struct).publish();
        componentPublisher = table.getStructArrayTopic("Componentes", Pose3d.struct).publish();
        basePublisher.set(BASE_POSE);
    }

    @Override
    public void close() {
        basePublisher.close();
        componentPublisher.close();
    }

    // #endregion

    // #region 🧭 POSES 3D DOS COMPONENTES

    /**
     * @param rollDegrees     rotação do manipulador (0 = como no desenho)
     * @param closingDegrees  quanto a garra fechou desde o 0 (toda aberta); NaN = 0
     * @param wheelRotations  voltas das rodas (+ = puxando)
     * @param objectHeld      a garra segura um objeto (desenhado entre as rodas)
     */
    public static Pose3d[] componentPoses(double rollDegrees, double closingDegrees,
            double wheelRotations, boolean objectHeld) {
        double roll = Double.isFinite(rollDegrees) ? Math.toRadians(rollDegrees) : 0.0;
        double wheels = Double.isFinite(wheelRotations) ? wheelRotations * 2.0 * Math.PI : 0.0;
        double closing = Double.isFinite(closingDegrees) ? Math.toRadians(closingDegrees) : 0.0;

        Pose3d body = new Pose3d(ROLL_AXIS, new Rotation3d(roll, 0.0, 0.0));
        // As peças do modelo estão como no desenho: primeiro cada braço vai do
        // ângulo do desenho até o 0 (toda aberta), depois fecha. Fechar gira o
        // braço direito para a esquerda (+) e o esquerdo para a direita (-).
        double rightTurn = Math.toRadians(CAD_RIGHT_ARM_DEGREES - CLAW_OPEN_ARM_DEGREES) + closing;
        double leftTurn = Math.toRadians(CLAW_OPEN_ARM_DEGREES - CAD_LEFT_ARM_DEGREES) - closing;
        Pose3d rightArm = body.transformBy(new Transform3d(RIGHT_PIVOT, new Rotation3d(0.0, 0.0, rightTurn)));
        Pose3d leftArm = body.transformBy(new Transform3d(LEFT_PIVOT, new Rotation3d(0.0, 0.0, leftTurn)));
        // Puxando, as rodas dos dois lados giram em sentidos opostos.
        Pose3d rightWheels = rightArm.transformBy(new Transform3d(RIGHT_AXLE, new Rotation3d(0.0, 0.0, wheels)));
        Pose3d leftWheels = leftArm.transformBy(new Transform3d(LEFT_AXLE, new Rotation3d(0.0, 0.0, -wheels)));
        return new Pose3d[] {body, rightArm, leftArm, rightWheels, leftWheels,
            objectHeld ? objectPose(closing) : HIDDEN_OBJECT};
    }

    /** Entre as rodas, no eixo da rotação: girar o manipulador não o move. */
    private static Pose3d objectPose(double closingRadians) {
        double arm = Math.toRadians(CLAW_OPEN_ARM_DEGREES) - closingRadians;
        double axleX = PIVOT_X_METERS + ARM_LENGTH_METERS * Math.cos(arm);
        return new Pose3d(axleX, 0.0, ROLL_AXIS_HEIGHT_METERS, new Rotation3d());
    }

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 📊 TELEMETRIA

    /**
     * Publica as poses das peças a partir do giro e fechamento em graus, voltas das rodas e presença
     * do objeto.
     */
    public void update(double rollDegrees, double closingDegrees, double wheelRotations, boolean objectHeld) {
        // O modelo fica parado na origem; só as peças se mexem.
        componentPublisher.set(componentPoses(rollDegrees, closingDegrees, wheelRotations, objectHeld));
    }

    // #endregion

    // #endregion
}
