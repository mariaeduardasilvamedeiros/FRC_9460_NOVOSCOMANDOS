package frc.robot.subsystems.manipulador;

// #region 📚 IMPORTS

import static frc.robot.subsystems.manipulador.ManipuladorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// #endregion

/**
 * Manipulador: garra que fecha por motor (a mola abre), rodas que puxam o
 * objeto e rotação no eixo longo. Garra e rodas: um Kraken X60 cada, os dois
 * em cima do manipulador, lado a lado. Rotação: Spark Flex com NEO Vortex.
 *
 * A garra fecha por FORÇA, porque o objeto ainda não é conhecido: vai até o
 * limite do mecanismo pelo Motion Magic; se parar antes, há um objeto preso,
 * e ela passa a segurar com tensão fixa. O ângulo em que parou dá a largura
 * do objeto. Abrir é soltar o motor (Coast): a mola volta a garra para o 0.
 * Fechada, ela continua fechada até alguém pedir para abrir (girar com o
 * objeto não solta nada).
 *
 * Usa os motores reais no roboRIO e modelos físicos na simulação.
 */
public class Manipulador extends SubsystemBase implements AutoCloseable {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    // #region 📦 IO E ESTADO

    /** O que a garra está fazendo. */
    public enum EstadoGarra { ABERTA, ABRINDO, FECHANDO, SEGURANDO_OBJETO, FECHADA_VAZIA }

    private final ManipuladorVisualizer visualizer = new ManipuladorVisualizer();
    private final ManipuladorGarraIO clawIO;
    private final ManipuladorRodasIO wheelsIO;
    private final ManipuladorGiroIO rotationIO;
    private final ManipuladorGarraIO.ManipuladorGarraIOInputs claw = new ManipuladorGarraIO.ManipuladorGarraIOInputs();
    private final ManipuladorRodasIO.ManipuladorRodasIOInputs wheels = new ManipuladorRodasIO.ManipuladorRodasIOInputs();
    private final ManipuladorGiroIO.ManipuladorGiroIOInputs rotation = new ManipuladorGiroIO.ManipuladorGiroIOInputs();

    private boolean closeRequested;
    private EstadoGarra clawState = EstadoGarra.ABERTA;
    private double clawStoppedSinceSeconds = Double.NaN;
    private double clawClosingSinceSeconds = Double.NaN;
    // 🛑 Tempo esgotado ao fechar: não fecha de novo até um novo pedido (resetClawFault).
    private boolean clawCloseBlocked;
    private String clawFault = "NENHUMA";
    private double wheelsTargetRpm;
    private String wheelsStatus = "PARADO";
    private double rotationGoalDegrees = Double.NaN;
    private String rotationStatus = "PARADO";
    private double rotationMoveStartSeconds = Double.NaN;
    private boolean rotationGoalReached;
    // 🛑 Tempo esgotado ao girar: nao gira de novo ate um novo comando (soltar e apertar o botao).
    private boolean rotationBlocked;
    private boolean rotationHolding;
    private String rotationFault = "NENHUMA";

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region ⚙️ CONSTRUÇÃO

    /**
     * Seleciona automaticamente os motores reais ou os três modelos simulados. No
     * robô, um motor com a chave *_MOTOR_ENABLED = false não é criado (fica desligado).
     */
    public Manipulador() {
        this(!RobotBase.isReal() ? new ManipuladorGarraIOSim()
                : CLAW_MOTOR_ENABLED ? new ManipuladorGarraIOTalonFX() : ManipuladorGarraIO.desligado(),
            !RobotBase.isReal() ? new ManipuladorRodasIOSim()
                : WHEELS_MOTOR_ENABLED ? new ManipuladorRodasIOTalonFX() : ManipuladorRodasIO.desligado(),
            !RobotBase.isReal() ? new ManipuladorGiroIOSim()
                : ROTATION_MOTOR_ENABLED ? new ManipuladorGiroIOSparkFlex() : ManipuladorGiroIO.desligado());
    }

    /** Inicializa as dependências e o estado do manipulador. */
    public Manipulador(ManipuladorGarraIO clawIO, ManipuladorRodasIO wheelsIO, ManipuladorGiroIO rotationIO) {
        this.clawIO = clawIO;
        this.wheelsIO = wheelsIO;
        this.rotationIO = rotationIO;
    }

    // #endregion

    // #region 🛠️ AUXILIARES

    /** Confere comunicação, zero e posição finita antes de comandar a garra. */
    private boolean clawReady() {
        return claw.connected && claw.zeroValid && Double.isFinite(claw.positionRotations);
    }

    /** Confere comunicação, aceitação do controle e leituras de rotação e corrente das rodas. */
    private boolean wheelsSensorsValid() {
        return wheels.connected && wheels.controlRequestOK && Double.isFinite(wheels.velocityRpm)
            && Double.isFinite(wheels.currentAmps);
    }

    /** Confere comunicação, zero e posição finita antes de girar o manipulador. */
    private boolean rotationReady() {
        return rotation.connected && rotation.zeroValid && Double.isFinite(rotation.positionRotations)
            && Double.isFinite(rotation.velocityRotationsPerSec);
    }

    /** Folga entre as rodas (borda a borda), em metros, com a garra fechada tantos graus. */
    static double wheelGapMeters(double closingDegrees) {
        double arm = Math.toRadians(CLAW_OPEN_ARM_DEGREES - closingDegrees);
        return 2.0 * (PIVOT_HALF_SPACING_METERS + ARM_LENGTH_METERS * Math.sin(arm) - WHEEL_RADIUS_METERS);
    }

    // #endregion

    // #region 🛠️ CONSULTAS

    /** Estado atual da garra, incluindo abertura, fechamento e retenção do objeto. */
    public EstadoGarra getClawState() {
        return clawState;
    }

    /** Quanto os braços fecharam desde o 0 (graus em cada braço). */
    public double getClawClosingDegrees() {
        return claw.positionRotations / CLAW_REDUCTION * 360.0;
    }

    /** A garra parou antes do limite: há um objeto preso. */
    public boolean hasObject() {
        return clawState == EstadoGarra.SEGURANDO_OBJETO;
    }

    /** Fechou: com objeto ou vazia (no limite). */
    public boolean isClawClosed() {
        return clawState == EstadoGarra.SEGURANDO_OBJETO || clawState == EstadoGarra.FECHADA_VAZIA;
    }

    /** Indica se a máquina de estados reconheceu a garra como aberta. */
    public boolean isClawOpen() {
        return clawState == EstadoGarra.ABERTA;
    }

    /** Largura do objeto preso (folga entre as rodas + o quanto elas afundam); NaN sem objeto. */
    public double getObjectWidthMeters() {
        return hasObject() ? wheelGapMeters(getClawClosingDegrees()) + WHEEL_SQUISH_METERS : Double.NaN;
    }

    /** RPM medido no motor das rodas do manipulador. */
    public double getWheelsRpm() {
        return wheels.velocityRpm;
    }

    /** Converte as voltas do motor em graus de rotação do manipulador. */
    public double getRotationDegrees() {
        return rotation.positionRotations / ROTATION_REDUCTION * 360.0;
    }

    /** Confere referência válida, meta finita e erro angular dentro da tolerância. */
    public boolean isRotationAtGoal() {
        return isRotationAtDegrees(rotationGoalDegrees);
    }

    /** Confirma a posicao solicitada e baixa velocidade, inclusive ao terminar uma sequencia. */
    public boolean isRotationAtDegrees(double degrees) {
        return !rotationBlocked && rotationReady() && Double.isFinite(degrees)
            && Math.abs(degrees - getRotationDegrees()) <= ROTATION_TOLERANCE_DEGREES
            && Math.abs(rotation.velocityRotationsPerSec / ROTATION_REDUCTION * 360.0)
                <= ROTATION_STOPPED_DEGREES_PER_SEC;
    }

    public boolean hasRotationFault() {
        return rotationBlocked;
    }

    public String getRotationFault() {
        return rotationFault;
    }

    /** Somente no inicio de um novo comando; nunca a cada ciclo do botao segurado. */
    public void resetRotationFault() {
        rotationBlocked = false;
        rotationHolding = false;
        rotationFault = "NENHUMA";
        rotationGoalReached = false;
        rotationMoveStartSeconds = Timer.getTimestamp();
    }

    // #endregion

    // #region 🚧 CONTROLE - GARRA (FECHA POR FORÇA ATÉ O LIMITE)

    /**
     * Fechando: Motion Magic até o limite. Chegou ao limite: fechou vazia.
     * Parou antes (mais devagar que CLAW_STOPPED_DEGREES_PER_SEC por
     * CLAW_STOPPED_SECONDS): objeto preso, segura com tensão fixa. O objeto
     * escorregou (a garra voltou a fechar): volta ao Motion Magic.
     */
    private void updateClaw(double now) {
        double closing = getClawClosingDegrees();
        if (!closeRequested) {
            clawStoppedSinceSeconds = Double.NaN;
            clawState = closing <= CLAW_POSITION_TOLERANCE_DEGREES ? EstadoGarra.ABERTA : EstadoGarra.ABRINDO;
            return;
        }
        double speed = Math.abs(claw.velocityRotationsPerSec / CLAW_REDUCTION * 360.0);
        if (closing >= CLAW_CLOSED_LIMIT_DEGREES - CLAW_POSITION_TOLERANCE_DEGREES) {
            clawState = EstadoGarra.FECHADA_VAZIA;
            clawStoppedSinceSeconds = Double.NaN;
            clawIO.setClosingTargetRotations(CLAW_CLOSED_LIMIT_MOTOR_ROTATIONS);
            return;
        }
        boolean stopped = closing > CLAW_POSITION_TOLERANCE_DEGREES && speed < CLAW_STOPPED_DEGREES_PER_SEC;
        if (clawState == EstadoGarra.SEGURANDO_OBJETO && speed < 3.0 * CLAW_STOPPED_DEGREES_PER_SEC) {
            clawIO.holdWithVolts(CLAW_HOLD_VOLTS);
            return;
        }
        clawStoppedSinceSeconds = stopped
            ? (Double.isNaN(clawStoppedSinceSeconds) ? now : clawStoppedSinceSeconds) : Double.NaN;
        if (stopped && now - clawStoppedSinceSeconds >= CLAW_STOPPED_SECONDS) {
            clawState = EstadoGarra.SEGURANDO_OBJETO;
            clawIO.holdWithVolts(CLAW_HOLD_VOLTS);
        } else {
            if (clawState != EstadoGarra.FECHANDO) clawClosingSinceSeconds = now;
            clawState = EstadoGarra.FECHANDO;
            // 🛑 Fechando há tempo demais, sem prender nem chegar ao limite: algo travou.
            if (now - clawClosingSinceSeconds > CLAW_CLOSE_TIMEOUT_SECONDS) {
                abortClose("TEMPO_ESGOTADO_AO_FECHAR");
                return;
            }
            clawIO.setClosingTargetRotations(CLAW_CLOSED_LIMIT_MOTOR_ROTATIONS);
        }
    }

    /** Solta a garra (a mola abre) e não fecha de novo até um novo pedido. */
    private void abortClose(String reason) {
        closeRequested = false;
        clawCloseBlocked = true;
        clawFault = reason;
        clawStoppedSinceSeconds = Double.NaN;
        clawIO.release();
        clawState = getClawClosingDegrees() <= CLAW_POSITION_TOLERANCE_DEGREES
            ? EstadoGarra.ABERTA : EstadoGarra.ABRINDO;
    }

    /** Fecha até o limite ou até prender o objeto; fica fechada até openClaw(). */
    public void closeClaw() {
        if (DriverStation.isDisabled() || !clawReady() || clawCloseBlocked) {
            closeRequested = false;
            clawIO.release();
            return;
        }
        if (!closeRequested) {
            closeRequested = true;
            clawState = EstadoGarra.FECHANDO;
            clawStoppedSinceSeconds = Double.NaN;
            clawClosingSinceSeconds = Timer.getTimestamp();
            clawIO.setClosingTargetRotations(CLAW_CLOSED_LIMIT_MOTOR_ROTATIONS);
        }
    }

    /** Novo pedido de fechar (o comando começou de novo): libera a garra depois de uma falha. */
    public void resetClawFault() {
        clawCloseBlocked = false;
        clawFault = "NENHUMA";
    }

    /** Motivo da última falha da garra ("NENHUMA" sem falha). */
    public String getClawFault() {
        return clawFault;
    }

    /** Solta o motor (Coast): a mola volta a garra para o 0. */
    public void openClaw() {
        closeRequested = false;
        clawIO.release();
        clawState = getClawClosingDegrees() <= CLAW_POSITION_TOLERANCE_DEGREES
            ? EstadoGarra.ABERTA : EstadoGarra.ABRINDO;
    }

    // #endregion

    // #region 🎯 CONTROLE - RODAS (PID DE VELOCIDADE NO TALONFX)

    /** Rodas puxando o objeto para dentro. */
    public void pullWheels() {
        if (DriverStation.isDisabled() || !wheelsSensorsValid()) {
            wheelsTargetRpm = 0.0;
            wheelsStatus = "PEDIDO_RECUSADO";
            wheelsIO.stop();
            return;
        }
        wheelsTargetRpm = MathUtil.clamp(INTAKE_RPM, -WHEELS_MAX_RPM, WHEELS_MAX_RPM);
        wheelsStatus = "PUXANDO";
        wheelsIO.setVelocityRpm(wheelsTargetRpm);
    }

    /** Neutro em Brake. */
    public void stopWheels() {
        wheelsTargetRpm = 0.0;
        wheelsStatus = "PARADO";
        wheelsIO.stop();
    }

    // #endregion

    // #region 🎯 CONTROLE - ROTAÇÃO (MAXMOTION NO SPARK FLEX)

    /** Meta em graus (0 = como no desenho), presa ao curso da fiação. */
    public void setRotationDegrees(double degrees) {
        if (DriverStation.isDisabled() || !rotationReady() || !Double.isFinite(degrees)) {
            rotationGoalDegrees = Double.NaN;
            rotationStatus = "PEDIDO_RECUSADO";
            rotationIO.stop();
            return;
        }
        if (rotationBlocked) {
            rotationIO.stop();
            return;
        }
        double goal = MathUtil.clamp(degrees, 0.0, ROTATION_END_DEGREES);
        if (goal != rotationGoalDegrees) {
            // Meta nova (ou depois de parar): o tempo máximo conta daqui.
            rotationMoveStartSeconds = Timer.getTimestamp();
            rotationGoalReached = false;
        }
        rotationGoalDegrees = goal;
        rotationHolding = false;
        rotationStatus = "MAXMOTION";
        rotationIO.setTargetRotations(rotationGoalDegrees / 360.0 * ROTATION_REDUCTION);
    }

    /** Direcional segurado: perfil de posicao em malha fechada ate o fim ou o zero. */
    public void rotateManual(int direction) {
        if (direction == 0) {
            holdRotationHere();
            return;
        }
        setRotationDegrees(direction > 0 ? ROTATION_END_DEGREES : 0.0);
    }

    /** Aplica neutro e preserva uma falha ate um novo pedido explicito. */
    public void stopRotation() {
        rotationGoalDegrees = Double.NaN;
        rotationStatus = rotationBlocked ? rotationFault : "PARADO";
        rotationHolding = false;
        rotationGoalReached = false;
        rotationIO.stop();
    }

    /**
     * Para o giro freando com a aceleração do perfil e segura onde parou. Com o
     * motor direto (1:1), desligar o motor quase não freia: o manipulador seguiria
     * girando por inércia até o batente. Depois de tempo esgotado (travado) ou sem
     * referência, só desliga.
     */
    public void holdRotationHere() {
        if (rotationBlocked || DriverStation.isDisabled() || !rotationReady()) {
            stopRotation();
            return;
        }
        // BACK e stopAll podem chamar varias vezes: capturar apenas uma vez.
        if (rotationHolding) return;
        if (isRotationAtGoal()) {
            rotationHolding = true;
            rotationStatus = "SEGURANDO";
            return;
        }
        double speedDegPerSec = rotation.velocityRotationsPerSec / ROTATION_REDUCTION * 360.0;
        double decelDegPerSec2 = ROTATION_ACCELERATION_ROTATIONS_PER_SEC2 / ROTATION_REDUCTION * 360.0;
        double stoppingDegrees = Math.copySign(speedDegPerSec * speedDegPerSec / (2.0 * decelDegPerSec2),
            speedDegPerSec);
        setRotationDegrees(getRotationDegrees() + stoppingDegrees);
        rotationHolding = true;
        rotationStatus = "FREANDO_E_SEGURANDO";
    }

    /** 🛑 Não chegou à meta em ROTATION_MOVE_TIMEOUT_SECONDS: para até soltar o botão. */
    private void checkRotationTimeout(double now) {
        if (!Double.isFinite(rotationGoalDegrees)) return;
        if (isRotationAtGoal()) {
            rotationGoalReached = true;
        } else {
            if (rotationGoalReached) {
                rotationMoveStartSeconds = now;
                rotationGoalReached = false;
            }
            if (now - rotationMoveStartSeconds > ROTATION_MOVE_TIMEOUT_SECONDS) {
                failRotation("TEMPO_ESGOTADO");
            }
        }
    }

    private void failRotation(String reason) {
        rotationBlocked = true;
        rotationFault = reason;
        stopRotation();
    }

    // #endregion

    // #region 🛑 SEGURANÇA - PARAR TUDO

    /**
     * Rodas paradas, rotação parada e garra solta (a mola abre). Habilitado, o
     * giro freia pelo perfil e segura (holdRotationHere); desabilitado, desliga.
     */
    public void stopAll() {
        stopWheels();
        holdRotationHere();
        openClaw();
    }

    // #endregion

    // #region 🧭 ZEROS

    /**
     * Só desabilitado. Garra aberta pela mola e manipulador como no desenho: as
     * posições atuais viram 0.
     */
    public void zeroHere() {
        if (DriverStation.isDisabled()) {
            stopAll();
            resetClawFault();
            resetRotationFault();
            clawIO.zeroHere();
            rotationIO.zeroHere();
            wheelsIO.resetWhileDisabled();
        }
    }

    // #endregion

    // #region 📡 1. SENSORES

    /** Lê as entradas dos três motores. */
    private void readSensors() {
        clawIO.updateInputs(claw);
        wheelsIO.updateInputs(wheels);
        rotationIO.updateInputs(rotation);
    }

    // #endregion

    // #region 🛑 2. SEGURANÇA - DESABILITADO, SENSORES E ZERO

    /** Interrompe partes sem referência ou desabilitadas e atualiza o estado de fechamento da garra. */
    private void applySafety() {
        boolean disabled = DriverStation.isDisabled();
        // Desabilitado ou sem leitura: solta a garra (a mola abre) e nada volta sozinho.
        if (disabled || !clawReady()) {
            closeRequested = false;
            clawIO.release();
        }
        if (disabled || !wheelsSensorsValid()) {
            wheelsTargetRpm = 0.0;
            wheelsStatus = disabled ? "DESABILITADO"
                : !wheels.controlRequestOK ? "FALHA_COMANDO_MOTOR" : "SENSOR_INDISPONIVEL";
            wheelsIO.stop();
        }
        // 🛑🧭 Sem zero, a rotação não sabe onde está a fiação: não se move.
        if (disabled || !rotationReady()) {
            if (!disabled && Double.isFinite(rotationGoalDegrees)) {
                failRotation("LEITURA_INVALIDA_REFACA_ZERO");
            }
            rotationGoalDegrees = Double.NaN;
            rotationHolding = false;
            rotationStatus = rotationBlocked ? rotationFault : disabled ? "DESABILITADO"
                : !rotation.connected ? "SENSOR_INDISPONIVEL" : "ZERO_PERDIDO";
            rotationIO.stop();
        }
        double now = Timer.getTimestamp();
        checkRotationTimeout(now);
        updateClaw(now);
    }

    // #endregion

    // #region 📊 3. TELEMETRIA (NOMES USADOS PELO LAYOUT FRC9460-Manipulador)

    /** Publica estados, metas e grandezas do mecanismo para os painéis e a análise dos logs. */
    private void publishTelemetry() {
        var command = getCurrentCommand();
        SmartDashboard.putString("Manipulador/Comando", command == null ? "Nenhum" : command.getName());

        double closing = getClawClosingDegrees();
        SmartDashboard.putBoolean("Manipulador/Garra/Conectado", claw.connected);
        SmartDashboard.putBoolean("Manipulador/Garra/ZeroValido", claw.zeroValid);
        SmartDashboard.putString("Manipulador/Garra/Zero", claw.startupZero);
        SmartDashboard.putBoolean("Manipulador/Rodas/Conectado", wheels.connected);
        SmartDashboard.putString("Manipulador/Rodas/Configuracao", wheels.configurationStatus);
        SmartDashboard.putBoolean("Manipulador/Giro/Conectado", rotation.connected);
        SmartDashboard.putBoolean("Manipulador/Giro/ZeroValido", rotation.zeroValid);
        SmartDashboard.putString("Manipulador/Giro/Zero", rotation.startupZero);
        SmartDashboard.putString("Manipulador/Garra/Estado", clawState.name());
        SmartDashboard.putString("Manipulador/Garra/Falha", clawFault);
        SmartDashboard.putBoolean("Manipulador/Garra/TemObjeto", hasObject());
        SmartDashboard.putNumber("Manipulador/Garra/FechamentoGraus", closing);
        SmartDashboard.putNumber("Manipulador/Garra/MetaGraus", closeRequested ? CLAW_CLOSED_LIMIT_DEGREES : Double.NaN);
        SmartDashboard.putNumber("Manipulador/Garra/ReferenciaGraus",
            claw.profileReferenceRotations / CLAW_REDUCTION * 360.0);
        SmartDashboard.putNumber("Manipulador/Garra/AberturaGraus", CLAW_OPEN_ARM_DEGREES - closing);
        SmartDashboard.putNumber("Manipulador/Garra/FolgaEntreRodasCm", wheelGapMeters(closing) * 100.0);
        SmartDashboard.putNumber("Manipulador/Garra/LarguraObjetoCm", getObjectWidthMeters() * 100.0);

        SmartDashboard.putNumber("Manipulador/Rodas/AlvoRpm", wheelsTargetRpm);
        SmartDashboard.putNumber("Manipulador/Rodas/Rpm", wheels.velocityRpm);
        SmartDashboard.putNumber("Manipulador/Rodas/RodaRpm", wheels.velocityRpm / WHEELS_REDUCTION);
        SmartDashboard.putNumber("Manipulador/Rodas/BordaMetrosPorSeg",
            wheels.velocityRpm / WHEELS_REDUCTION / 60.0 * 2.0 * Math.PI * WHEEL_RADIUS_METERS);
        SmartDashboard.putString("Manipulador/Rodas/Estado", wheelsStatus);

        SmartDashboard.putNumber("Manipulador/Giro/AnguloGraus", getRotationDegrees());
        SmartDashboard.putNumber("Manipulador/Giro/MetaGraus", rotationGoalDegrees);
        SmartDashboard.putNumber("Manipulador/Giro/ReferenciaGraus",
            rotation.profileReferenceRotations / ROTATION_REDUCTION * 360.0);
        SmartDashboard.putBoolean("Manipulador/Giro/NaMeta", isRotationAtGoal());
        SmartDashboard.putString("Manipulador/Giro/Estado", rotationStatus);
        SmartDashboard.putString("Manipulador/Giro/Falha", rotationFault);
        SmartDashboard.putNumber("Manipulador/Giro/VelocidadeGrausPorSegundo",
            rotation.velocityRotationsPerSec / ROTATION_REDUCTION * 360.0);
        SmartDashboard.putNumber("Manipulador/Giro/ErroGraus", rotationGoalDegrees - getRotationDegrees());
        SmartDashboard.putNumber("Manipulador/Giro/Tensao", rotation.appliedVolts);

        SmartDashboard.putNumber("Manipulador/Garra/Inputs/CurrentAmps", claw.currentAmps);
        SmartDashboard.putNumber("Manipulador/Rodas/Inputs/CurrentAmps", wheels.currentAmps);
        SmartDashboard.putNumber("Manipulador/Giro/Inputs/CurrentAmps", rotation.currentAmps);

        visualizer.update(getRotationDegrees(), closing,
            wheels.positionRotations / WHEELS_REDUCTION, hasObject());
    }

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 🧹 ENCERRAMENTO (TESTES)

    /** Libera os recursos usados pelo subsistema e o remove do agendador ao encerrar os testes. */
    @Override
    public void close() {
        stopAll();
        clawIO.close();
        wheelsIO.close();
        rotationIO.close();
        visualizer.close();
        CommandScheduler.getInstance().unregisterSubsystem(this);
    }

    // #endregion

    // #region 🔁 PERIODIC - ORDEM DE CADA CICLO (20 ms)

    // Depois do periodic, os comandos (pasta commands/manipulador) chamam as
    // funções de CONTROLE da parte 2: closeClaw, pullWheels, setRotationDegrees...
    @Override
    public void periodic() {
        readSensors();
        applySafety();
        publishTelemetry();
    }

    // #endregion

    // #endregion
}
