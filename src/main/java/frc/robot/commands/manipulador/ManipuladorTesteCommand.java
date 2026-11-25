package frc.robot.commands.manipulador;

// #region 📚 IMPORTS

import static frc.robot.subsystems.manipulador.ManipuladorConstants.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.manipulador.Manipulador;

// #endregion

/**
 * Sequência automática do manipulador, montada com os comandos de cada ação
 * disponível na simulação em SmartDashboard/Manipulador/TesteCompleto, na ordem:
 *
 * 1. GIRANDO_PARA_ZERO   ManipuladorGirarParaCommand(0°)
 * 2. FECHANDO            ManipuladorFecharCommand (rodas puxam, garra fecha)
 * 3. SEGURANDO           garra fechada, rodas paradas
 * 4. GIRANDO_90          ManipuladorGirarParaCommand(fim medido, ~90°), segurando
 * 5. VOLTANDO_PARA_ZERO  ManipuladorGirarParaCommand(0°)
 * 6. ABRINDO             ManipuladorAbrirCommand (a mola abre)
 *
 * Uma etapa que não conclui no tempo encerra o teste com o motivo
 * (GARRA_NAO_FECHOU, GIRO_NAO_CHEGOU_90, GARRA_NAO_ABRIU). Interrompida,
 * para tudo e solta a garra. Etapa e resultado ficam em Manipulador/Teste/*;
 * se pegou objeto e a largura dele ficam em Manipulador/Garra/*.
 *
 * A sequência fica dentro deste comando (como o WrapperCommand da WPILib):
 * assim o fim, normal ou interrompido, sempre para os motores e grava o motivo.
 */
public class ManipuladorTesteCommand extends Command {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    // #region 📦 ESTADO

    private static final String IN_PROGRESS = "EM_ANDAMENTO";

    private final Manipulador manipulador;
    private final Command sequencia;
    private String etapaAtual = "NAO_INICIADO";
    private String resultado = "NAO_INICIADO";

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region ⚙️ CONSTRUÇÃO

    public ManipuladorTesteCommand(Manipulador manipulador) {
        this.manipulador = manipulador;
        sequencia = Commands.sequence(
            Commands.runOnce(() -> resultado = IN_PROGRESS),
            girar("GIRANDO_PARA_ZERO", 0.0),
            etapa("FECHANDO", new ManipuladorFecharCommand(manipulador).withTimeout(TEST_CLOSE_TIMEOUT_SECONDS)),
            confere(manipulador::isClawClosed, "GARRA_NAO_FECHOU"),
            etapa("SEGURANDO", Commands.waitSeconds(TEST_HOLD_SECONDS)),
            girar("GIRANDO_90", TEST_ROTATION_DEGREES),
            girar("VOLTANDO_PARA_ZERO", 0.0),
            etapa("ABRINDO", new ManipuladorAbrirCommand(manipulador).withTimeout(TEST_OPEN_TIMEOUT_SECONDS)),
            confere(manipulador::isClawOpen, "GARRA_NAO_ABRIU"),
            Commands.runOnce(() -> conclui("OK")).onlyIf(this::emAndamento));
        // A sequência só roda por aqui; o agendador recusa usá-la sozinha.
        CommandScheduler.getInstance().registerComposedCommands(sequencia);
        addRequirements(sequencia.getRequirements());
        setName("Manipulador - teste completo");
    }

    // #endregion

    // #region 🛠️ AUXILIARES

    private boolean emAndamento() {
        return IN_PROGRESS.equals(resultado);
    }

    private void conclui(String result) {
        resultado = result;
        log();
    }

    private void marca(String nome) {
        etapaAtual = nome;
        log();
    }

    private void log() {
        SmartDashboard.putString("Manipulador/Teste/Etapa", etapaAtual);
        SmartDashboard.putString("Manipulador/Teste/Resultado", resultado);
    }

    /** Resultado do último teste: OK, GARRA_NAO_FECHOU, GIRO_NAO_CHEGOU_..., INTERROMPIDO_EM_... */
    public String getResultado() {
        return resultado;
    }

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 🎮 CICLO DO COMANDO (CONDUZ A SEQUÊNCIA)

    @Override
    public void initialize() {
        sequencia.initialize();
    }

    @Override
    public void execute() {
        sequencia.execute();
    }

    @Override
    public boolean isFinished() {
        return sequencia.isFinished();
    }

    // #endregion

    // #region 🧹 ENCERRAMENTO

    @Override
    public void end(boolean interrupted) {
        sequencia.end(interrupted);
        manipulador.stopAll();
        if (interrupted && emAndamento()) {
            resultado = "INTERROMPIDO_EM_" + etapaAtual;
        }
        marca("CONCLUIDO");
    }

    // #endregion

    // #endregion

    // #region 🎮 PARTE 4 - COMANDOS

    // #region 🛠️ PEÇAS DA SEQUÊNCIA

    /** Gira até o ângulo (com tempo máximo) e confere se chegou. */
    private Command girar(String nome, double degrees) {
        return etapa(nome, Commands.sequence(
            new ManipuladorGirarParaCommand(manipulador, degrees).withTimeout(TEST_ROTATION_TIMEOUT_SECONDS),
            Commands.runOnce(() -> {
                if (!manipulador.isRotationAtDegrees(degrees)) conclui("GIRO_NAO_CHEGOU_" + Math.round(degrees));
            })));
    }

    /** Uma etapa só roda se as anteriores deram certo. */
    private Command etapa(String nome, Command acao) {
        return Commands.sequence(Commands.runOnce(() -> marca(nome)), acao).onlyIf(this::emAndamento);
    }

    private Command confere(BooleanSupplier condicao, String falha) {
        return Commands.runOnce(() -> {
            if (!condicao.getAsBoolean()) conclui(falha);
        }).onlyIf(this::emAndamento);
    }

    // #endregion

    // #endregion
}
