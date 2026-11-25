package frc.robot.subsystems.swerve;

// #region 📚 IMPORTS

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

// #endregion

/**
 * Pigeon 2 (CTRE) no centro do robô, ID GYRO_ID. Lê o yaw e a velocidade de giro a
 * 100 Hz. Ao ligar, o yaw vira 0: quem define para onde é "a frente do campo" é o
 * Swerve (zeroHeading / resetPose), sem mexer no Pigeon.
 *
 * Firmware incompatível com o Phoenix 6 2026: avisa uma vez e para de chamar o Pigeon
 * (senão a Driver Station enche de erros -1003/-1008 e esconde as outras mensagens).
 */
public final class SwerveGyroIOPigeon2 implements SwerveGyroIO {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    // Sem Pigeon ao ligar: tenta configurar de novo a cada 5 s, só desabilitado
    // (configurar trava o ciclo por alguns ms).
    private static final double RETRY_SECONDS = 5.0;

    private final Pigeon2 pigeon = new Pigeon2(GYRO_ID);
    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
    private final BaseStatusSignal[] signals = {yaw, yawVelocity};
    private boolean configured;
    // Firmware que o Phoenix 6 2026 não aceita: só volta com atualização + reiniciar o código.
    private boolean firmwareProblem;
    private String configurationStatus = "NAO_CONFIGURADO";
    private double lastAttemptSeconds;

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region ⚙️ CONSTRUÇÃO E CONFIGURAÇÃO

    public SwerveGyroIOPigeon2() {
        configure();
    }

    /** Montagem, yaw = 0, frequência dos sinais e desliga o que não é usado. */
    private void configure() {
        lastAttemptSeconds = Timer.getTimestamp();
        var config = new Pigeon2Configuration();
        config.MountPose.MountPoseYaw = GYRO_MOUNT_YAW_DEGREES;
        config.MountPose.MountPosePitch = GYRO_MOUNT_PITCH_DEGREES;
        config.MountPose.MountPoseRoll = GYRO_MOUNT_ROLL_DEGREES;
        StatusCode result = pigeon.getConfigurator().apply(config);
        if (result.isOK()) result = pigeon.setYaw(0.0);
        if (result.isOK()) result = BaseStatusSignal.setUpdateFrequencyForAll(GYRO_SIGNAL_HZ, signals);
        if (result.isOK()) result = pigeon.optimizeBusUtilization();
        configured = result.isOK();
        firmwareProblem = result == StatusCode.FirmwareTooOld || result == StatusCode.FirmwareVersNotCompatible
            || result == StatusCode.FirmwareTooNew || result == StatusCode.FirmwareNonFRC;
        configurationStatus = configured ? "OK" : (firmwareProblem ? "FIRMWARE_" : "FALHA_CONFIG_") + result;
        if (configured) return;
        if (firmwareProblem) {
            DriverStation.reportError("Swerve: Pigeon 2 (CAN " + GYRO_ID + ") com firmware que o Phoenix 6 2026"
                + " não aceita (" + result + "). Atualizar no Phoenix Tuner X e reiniciar o código. Até lá a"
                + " frente vem das rodas e o código não chama mais o Pigeon (os erros -1003/-1008 param).", false);
        } else {
            DriverStation.reportWarning("Swerve: Pigeon 2 (CAN " + GYRO_ID + ") não respondeu (" + result
                + "). A frente do robô vem só das rodas até o Pigeon voltar.", false);
        }
    }

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 📡 LEITURAS

    @Override
    public void updateInputs(SwerveGyroIOInputs inputs) {
        if (!configured && !firmwareProblem && DriverStation.isDisabled()
                && Timer.getTimestamp() - lastAttemptSeconds >= RETRY_SECONDS) {
            configure();
        }
        if (!configured) {
            // Sem configurar não lê: cada leitura falhada vira um erro na Driver Station.
            inputs.connected = false;
            inputs.status = configurationStatus;
            return;
        }
        boolean ok = BaseStatusSignal.refreshAll(signals).isOK();
        for (var signal : signals) {
            ok &= signal.getStatus().isOK() && Double.isFinite(signal.getValueAsDouble())
                && signal.getTimestamp().isValid()
                && signal.getTimestamp().getLatency() <= MAX_SIGNAL_AGE_SECONDS;
        }
        inputs.connected = ok;
        inputs.status = ok ? "OK" : "SEM_LEITURA";
        inputs.yawRad = Units.degreesToRadians(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    }

    // #endregion

    // #region 🧹 ENCERRAMENTO

    @Override
    public void close() {
        pigeon.close();
    }

    // #endregion

    // #endregion
}
