package frc.robot.commands.manipulador;

// #region 📚 IMPORTS

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.manipulador.Manipulador;

// #endregion

/**
 * Gira o manipulador até um ângulo (graus, 0 = como no desenho) e termina
 * quando chega. Chegou: o MAXMotion do Spark Flex continua segurando o ângulo.
 * Interrompido (soltou o botão no meio): freia pelo perfil e segura onde parou,
 * porque com o motor direto o Brake quase não freia. As rodas não mudam.
 */
public class ManipuladorGirarParaCommand extends Command {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    // #region 🛠️ DEPENDÊNCIAS

    private final Manipulador manipulador;
    private final double degrees;

    // #endregion

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    // #region ⚙️ CONSTRUÇÃO

    public ManipuladorGirarParaCommand(Manipulador manipulador, double degrees) {
        this.manipulador = manipulador;
        this.degrees = degrees;
        addRequirements(manipulador);
        setName("Manipulador - girar para " + degrees + "°");
    }

    // #endregion

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    // #region 🎮 CICLO DO COMANDO

    // Manipulador: setRotationDegrees, isRotationAtGoal e stopRotation.
    // =========================================================================

    // VEZ, execute E isFinished A CADA 20 ms, E end NO FIM)
    // =========================================================================
    @Override
    public void initialize() {
        manipulador.resetRotationFault();
    }

    @Override
    public void execute() {
        manipulador.setRotationDegrees(degrees);
    }

    @Override
    public boolean isFinished() {
        return manipulador.hasRotationFault() || manipulador.isRotationAtGoal();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            manipulador.holdRotationHere();
        }
    }

    // #endregion

    // #endregion
}
