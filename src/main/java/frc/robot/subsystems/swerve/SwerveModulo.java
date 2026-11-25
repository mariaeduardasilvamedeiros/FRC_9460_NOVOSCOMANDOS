package frc.robot.subsystems.swerve;

// #region 📚 IMPORTS

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// #endregion

/**
 * Lógica de um módulo (a mesma do MAXSwerveModule do template REV), em cima de
 * qualquer SwerveIO: otimiza o estado pedido e mantém a odometria contínua se o Spark
 * da tração reiniciar. O offset do encoder absoluto fica dentro do IO.
 *
 * Ângulos públicos: no referencial do chassi (0 = roda apontando para a frente do robô).
 */
public final class SwerveModulo implements AutoCloseable {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    private final String name;
    private final SwerveIO io;
    private final SwerveIO.SwerveIOInputs inputs = new SwerveIO.SwerveIOInputs();
    // Último estado mandado ao IO, já otimizado (referencial do chassi).
    private final SwerveModuleState desiredState = new SwerveModuleState();

    // 🛑 Spark da tração reiniciou: a posição dele voltou a 0. Esta correção mantém a
    // distância percorrida contínua para a odometria.
    private double drivePositionCorrectionMeters;
    private double lastRawDrivePositionMeters = Double.NaN;
    private int driveResetCount;

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region ⚙️ CONSTRUÇÃO

    SwerveModulo(String name, SwerveIO io) {
        this.name = name;
        this.io = io;
    }

    // #endregion

    // #region 🛠️ CONSULTAS

    public String getName() {
        return name;
    }

    /** Ângulo da roda em relação ao chassi (0 = para a frente, anti-horário positivo). */
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(inputs.turnPositionRad);
    }

    /** Distância percorrida pela roda desde que o código ligou (m), sem os saltos de reinício. */
    public double getDrivePositionMeters() {
        return inputs.drivePositionMeters + drivePositionCorrectionMeters;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(inputs.driveVelocityMetersPerSec, getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getAngle());
    }

    public SwerveModuleState getDesiredState() {
        return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }

    /** Os dois Spark configurados e respondendo, e o giro alinhado pelo absoluto. */
    public boolean isConnected() {
        return inputs.driveConnected && inputs.turnConnected;
    }

    public int getDriveResetCount() {
        return driveResetCount;
    }

    /** Leituras cruas do IO (telemetria e testes). */
    public SwerveIO.SwerveIOInputs getInputs() {
        return inputs;
    }

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 📡 LEITURAS

    void updateInputs() {
        io.updateInputs(inputs);
        if (!inputs.driveConnected) return;
        double raw = inputs.drivePositionMeters;
        if (Double.isFinite(lastRawDrivePositionMeters)
                && Math.abs(raw - lastRawDrivePositionMeters) > MAX_DRIVE_STEP_METERS) {
            drivePositionCorrectionMeters += lastRawDrivePositionMeters - raw;
            driveResetCount++;
        }
        lastRawDrivePositionMeters = raw;
    }

    // #endregion

    // #region 🎮 COMANDOS (O SWERVE CHAMA)

    /**
     * Estado no referencial do chassi. Como no template REV: otimiza (nunca gira mais
     * de 90°, invertendo a roda se precisar). A mais, como
     * recomenda a WPILib: reduz a velocidade pelo cosseno do erro de ângulo, para a
     * roda não empurrar para o lado errado enquanto ainda está girando.
     */
    void setDesiredState(SwerveModuleState state) {
        var current = getAngle();
        var optimized = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
        optimized.optimize(current);
        optimized.cosineScale(current);
        desiredState.speedMetersPerSecond = optimized.speedMetersPerSecond;
        desiredState.angle = optimized.angle;
        io.setDesiredState(optimized.speedMetersPerSecond, optimized.angle.getRadians());
    }

    /**
     * Roda parada apontando EXATAMENTE neste ângulo, sem a otimização (que deixaria a roda
     * a 180°, invertida). Usado para alinhar as rodas em 0° (botão START).
     */
    void setAngleWithoutOptimizing(Rotation2d angle) {
        desiredState.speedMetersPerSecond = 0.0;
        desiredState.angle = angle;
        io.setDesiredState(0.0, angle.getRadians());
    }

    /** Neutro nos dois motores (Brake). */
    void stop() {
        desiredState.speedMetersPerSecond = 0.0;
        io.stop();
    }

    // #endregion

    // #region 🧹 ENCERRAMENTO

    @Override
    public void close() {
        io.stop();
        io.close();
    }

    // #endregion

    // #endregion
}
