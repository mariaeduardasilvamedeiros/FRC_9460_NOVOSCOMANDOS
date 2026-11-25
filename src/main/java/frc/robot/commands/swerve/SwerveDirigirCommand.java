package frc.robot.commands.swerve;

// #region 📚 IMPORTS

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;  

import frc.robot.subsystems.swerve.Swerve;

// #endregion

/**
 * Comando padrão do swerve. Orientada ao campo (padrão), vista da Driver Station:
 * empurrar o analógico para a frente leva o robô para longe da Driver Station, para
 * qualquer lado que a frente dele esteja apontando, nas duas alianças. Orientada ao
 * robô (botão Y alterna): para a frente = para a frente do robô.
 *
 * Zona morta em círculo, entrada ao quadrado (mais precisão devagar), modo lento (LT,
 * se houver trigger), velocidade do piloto (padrão 50%; B 80%, X 30%:
 * Swerve.getDriverSpeedScale), frente do piloto (Start + A: Swerve.getDriverHeading) e
 * limite de aceleração no pedido. Só pilota no Teleop habilitado: no autônomo (sem outro
 * comando) mantém o robô parado com as rodas no lugar.
 */
public class SwerveDirigirCommand extends Command {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    private static final double LOOP_SECONDS = 0.02;

    private final Swerve swerve;
    private final DoubleSupplier forward;
    private final DoubleSupplier left;
    private final DoubleSupplier counterClockwise;
    private final BooleanSupplier slow;

    // Último pedido depois do limite de aceleração (referencial da Driver Station no modo
    // campo; referencial do robô no modo robô).
    private double lastVx;
    private double lastVy;
    private double lastOmega;
    private boolean lastFieldCentric;

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region ⚙️ CONSTRUÇÃO

    /**
     * @param forward          +1 = para longe da Driver Station (analógico esquerdo para cima)
     * @param left             +1 = para a esquerda do piloto
     * @param counterClockwise +1 = girar anti-horário visto de cima
     * @param slow             segurado = modo lento (TELEOP_SLOW_SCALE)
     */
    public SwerveDirigirCommand(Swerve swerve, DoubleSupplier forward, DoubleSupplier left,
            DoubleSupplier counterClockwise, BooleanSupplier slow) {
        this.swerve = swerve;
        this.forward = forward;
        this.left = left;
        this.counterClockwise = counterClockwise;
        this.slow = slow;
        addRequirements(swerve);
        setName("Swerve - dirigir");
    }

    // #endregion

    // #region 🛠️ AUXILIARES

    /** Zona morta no vetor (círculo): a diagonal não "gruda" nos eixos. Saída ao quadrado. */
    static double[] shapeTranslation(double x, double y) {
        double magnitude = MathUtil.applyDeadband(Math.min(1.0, Math.hypot(x, y)), JOYSTICK_DEADBAND);
        if (magnitude == 0.0) return new double[] {0.0, 0.0};
        double angle = Math.atan2(y, x);
        magnitude *= magnitude;
        return new double[] {magnitude * Math.cos(angle), magnitude * Math.sin(angle)};
    }

    static double shapeRotation(double value) {
        double r = MathUtil.applyDeadband(MathUtil.clamp(value, -1.0, 1.0), JOYSTICK_DEADBAND);
        return Math.copySign(r * r, r);
    }

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    @Override
    public void initialize() {
        startFromCurrentSpeed(swerve.isFieldCentric());
    }

    /**
     * Começa da velocidade atual, no referencial do modo (campo visto da DS ou robô):
     * sem tranco ao começar e ao trocar de modo com o robô andando.
     */
    private void startFromCurrentSpeed(boolean fieldCentric) {
        var speeds = swerve.getRobotRelativeSpeeds();
        lastFieldCentric = fieldCentric;
        lastOmega = speeds.omegaRadiansPerSecond;
        if (!fieldCentric) {
            lastVx = speeds.vxMetersPerSecond;
            lastVy = speeds.vyMetersPerSecond;
            return;
        }
        // Frente do piloto (Start + A), a mesma usada para dirigir no modo campo.
        var field = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, swerve.getDriverHeading());
        double sign = Swerve.isRedAlliance() ? -1.0 : 1.0;
        lastVx = sign * field.vxMetersPerSecond;
        lastVy = sign * field.vyMetersPerSecond;
    }

    @Override
    public void execute() {
        if (!DriverStation.isTeleopEnabled()) {
            lastVx = 0.0;
            lastVy = 0.0;
            lastOmega = 0.0;
            swerve.runVelocity(new ChassisSpeeds());
            return;
        }
        boolean fieldCentric = swerve.isFieldCentric();
        if (fieldCentric != lastFieldCentric) startFromCurrentSpeed(fieldCentric);
        // Modo lento por trigger (se houver) e velocidade do piloto (B/X) se somam.
        double scale = (slow.getAsBoolean() ? TELEOP_SLOW_SCALE : 1.0) * swerve.getDriverSpeedScale();
        double[] translation = shapeTranslation(forward.getAsDouble(), left.getAsDouble());
        double vx = translation[0] * TELEOP_MAX_SPEED_MPS * scale;
        double vy = translation[1] * TELEOP_MAX_SPEED_MPS * scale;
        double omega = shapeRotation(counterClockwise.getAsDouble()) * TELEOP_MAX_ANGULAR_SPEED_RAD_PER_SEC * scale;

        // 🛑 Limite de aceleração no vetor (mantém a direção pedida) e no giro.
        double dvx = vx - lastVx;
        double dvy = vy - lastVy;
        double dv = Math.hypot(dvx, dvy);
        double maxDv = TELEOP_MAX_ACCELERATION_MPS2 * LOOP_SECONDS;
        if (dv > maxDv) {
            dvx *= maxDv / dv;
            dvy *= maxDv / dv;
        }
        lastVx += dvx;
        lastVy += dvy;
        double maxDomega = TELEOP_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC2 * LOOP_SECONDS;
        lastOmega += MathUtil.clamp(omega - lastOmega, -maxDomega, maxDomega);

        if (!fieldCentric) {
            // Orientada ao robô: não depende da frente do robô no campo (útil sem o Pigeon).
            swerve.runVelocity(new ChassisSpeeds(lastVx, lastVy, lastOmega));
            return;
        }
        // Na aliança vermelha a Driver Station fica do outro lado do campo: "para a
        // frente do piloto" é o −X do campo (origem sempre no lado azul).
        // Usa a frente do piloto (Start + A) e não o heading da odometria: assim o ajuste
        // muda só a pilotagem e o driveFieldRelative do autônomo continua igual.
        double sign = Swerve.isRedAlliance() ? -1.0 : 1.0;
        swerve.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(sign * lastVx, sign * lastVy, lastOmega), swerve.getDriverHeading()));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.runVelocity(new ChassisSpeeds());
    }

    // #endregion
}