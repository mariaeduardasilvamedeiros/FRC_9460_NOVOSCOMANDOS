package frc.robot.commands.manipulador;

// #region 📚 IMPORTS

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.manipulador.Manipulador;

// #endregion

/**
 * Solta o objeto: o motor da garra fica solto (Coast) e a mola volta a garra
 * para o 0. As rodas param. Termina quando a garra está aberta.
 */
public class ManipuladorAbrirCommand extends Command {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    // #region 🛠️ DEPENDÊNCIAS

    private final Manipulador manipulador;

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region ⚙️ CONSTRUÇÃO

    public ManipuladorAbrirCommand(Manipulador manipulador) {
        this.manipulador = manipulador;
        addRequirements(manipulador);
        setName("Manipulador - abrir garra");
    }

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 🎮 CICLO DO COMANDO

    // Manipulador: stopWheels, openClaw e isClawOpen.
    // =========================================================================

    // VEZ, execute E isFinished A CADA 20 ms, E end NO FIM)
    // =========================================================================
    @Override
    public void initialize() {
        manipulador.stopWheels();
        manipulador.openClaw();
    }

    @Override
    public boolean isFinished() {
        return manipulador.isClawOpen();
    }

    // #endregion

    // #endregion
}
