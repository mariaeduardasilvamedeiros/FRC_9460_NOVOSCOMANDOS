package frc.robot.subsystems.swerve;

// #region 📚 IMPORTS

import edu.wpi.first.math.MathUtil;
import java.util.Arrays;

// #endregion

/**
 * Pega dois DIOs lendo o MESMO encoder absoluto (fio emendado, pino S encostado ou o cabo
 * de um módulo ligado em duas portas). Foi o que aconteceu com a TD em 19/11/2026: o DIO 3
 * leu 156,04° com a FE (DIO 0) em 156,06°.
 *
 * Dois encoders de verdade só ficam iguais por acaso e parados. Iguais (diferença menor que
 * SAME_DEGREES) enquanto giram juntos por TRAVEL_DEGREES = é o mesmo sinal. Só avisa: quem
 * decide qual cabo está errado é a equipe (girando um módulo de cada vez com a mão).
 */
final class SwerveSinalRepetido {

    // #region 📦 PARTE 1 - VARIÁVEIS E CONFIGURAÇÃO

    private static final double SAME_RAD = Math.toRadians(0.3);
    private static final double TRAVEL_RAD = Math.toRadians(20.0);

    private final int count;
    private final double[] lastRaw;
    private final double[] sameTravelRad; // por par (i < j)
    private final boolean[] found;

    // #endregion

    // #region 🛠️ PARTE 2 - FUNÇÕES

    SwerveSinalRepetido(int moduleCount) {
        count = moduleCount;
        lastRaw = new double[moduleCount];
        Arrays.fill(lastRaw, Double.NaN);
        sameTravelRad = new double[moduleCount * moduleCount];
        found = new boolean[moduleCount * moduleCount];
    }

    /** O par (i, j) já foi pego lendo o mesmo sinal. */
    boolean isRepeated(int i, int j) {
        return found[Math.min(i, j) * count + Math.max(i, j)];
    }

    // #endregion

    // #region 🔁 PARTE 3 - LÓGICA

    /**
     * Um ciclo (20 ms).
     *
     * @param rawRad leitura de cada absoluto (rad, qualquer faixa); NaN = sem sinal
     * @return índice i * count + j do par recém-pego (para avisar uma vez só); −1 = nenhum
     */
    int update(double[] rawRad) {
        int newPair = -1;
        for (int i = 0; i < count; i++) {
            for (int j = i + 1; j < count; j++) {
                int pair = i * count + j;
                double a = rawRad[i];
                double b = rawRad[j];
                if (!Double.isFinite(a) || !Double.isFinite(b)
                        || Math.abs(MathUtil.angleModulus(a - b)) > SAME_RAD) {
                    sameTravelRad[pair] = 0.0;
                    continue;
                }
                if (Double.isFinite(lastRaw[i])) sameTravelRad[pair] += Math.abs(MathUtil.angleModulus(a - lastRaw[i]));
                if (!found[pair] && sameTravelRad[pair] >= TRAVEL_RAD) {
                    found[pair] = true;
                    newPair = pair;
                }
            }
        }
        System.arraycopy(rawRad, 0, lastRaw, 0, count);
        return newPair;
    }

    // #endregion
}
