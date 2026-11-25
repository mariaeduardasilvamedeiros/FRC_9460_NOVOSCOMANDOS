package frc.robot.commands.manipulador;

// #region 📚 IMPORTS

import static frc.robot.subsystems.manipulador.ManipuladorConstants.WHEELS_PULL_AFTER_CLOSE_SECONDS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.manipulador.Manipulador;

// #endregion

/**
 * Pega o objeto: as rodas puxam e a garra fecha (por força, até o limite ou
 * até travar no objeto). Depois de fechar, as rodas seguem puxando por
 * WHEELS_PULL_AFTER_CLOSE_SECONDS para assentar o objeto, e o comando
 * termina. Terminou ou soltou o botão: as rodas param e a garra CONTINUA
 * fechada (girar com o objeto não solta nada). Quem solta é o
 * ManipuladorAbrirCommand.
 */
public class ManipuladorFecharCommand extends Command {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    // #region 🛠️ DEPENDÊNCIAS

    private final Manipulador manipulador;
    private double closedSinceSeconds = Double.NaN;

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region ⚙️ CONSTRUÇÃO

    public ManipuladorFecharCommand(Manipulador manipulador) {
        this.manipulador = manipulador;
        addRequirements(manipulador);
        setName("Manipulador - pegar e fechar");
    }

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 🎮 CICLO DO COMANDO

    // Manipulador: closeClaw, pullWheels, isClawClosed e stopWheels.
    // =========================================================================

    // VEZ, execute E isFinished A CADA 20 ms, E end NO FIM)
    // =========================================================================
    @Override
    public void initialize() {
        closedSinceSeconds = Double.NaN;
        // Apertar → de novo tenta outra vez depois de um tempo esgotado.
        manipulador.resetClawFault();
    }

    @Override
    public void execute() {
        manipulador.closeClaw();
        // Pedido a cada ciclo: alimenta o MotorSafety das rodas.
        manipulador.pullWheels();
        if (manipulador.isClawClosed() && Double.isNaN(closedSinceSeconds)) {
            closedSinceSeconds = Timer.getTimestamp();
        }
    }

    @Override
    public boolean isFinished() {
        return !Double.isNaN(closedSinceSeconds)
            && Timer.getTimestamp() - closedSinceSeconds >= WHEELS_PULL_AFTER_CLOSE_SECONDS;
    }

    @Override
    public void end(boolean interrupted) {
        manipulador.stopWheels();
    }

    // #endregion

    // #endregion
}
