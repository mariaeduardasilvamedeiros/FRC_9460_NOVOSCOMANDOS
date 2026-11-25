package frc.robot.commands.manipulador;

// #region 📚 IMPORTS

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.manipulador.Manipulador;

// #endregion

/** Direcional segurado: posicao com MAXMotion. Ao soltar, freia e sustenta pelo encoder. */
public class ManipuladorGirarManualCommand extends Command {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    private final Manipulador manipulador;
    private final int direction;

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    public ManipuladorGirarManualCommand(Manipulador manipulador, int direction) {
        this.manipulador = manipulador;
        this.direction = (int) Math.signum(direction);
        addRequirements(manipulador);
        setName(this.direction > 0 ? "Manipulador - girar manual (fim)" : "Manipulador - girar manual (0)");
    }

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // Pedido a cada ciclo: confere fim, 0 e velocidade com a leitura nova.
    @Override
    public void initialize() {
        manipulador.resetRotationFault();
    }

    @Override
    public boolean isFinished() {
        return manipulador.hasRotationFault();
    }

    @Override
    public void execute() {
        manipulador.rotateManual(direction);
    }

    @Override
    public void end(boolean interrupted) {
        manipulador.holdRotationHere();
    }

    // #endregion
}
