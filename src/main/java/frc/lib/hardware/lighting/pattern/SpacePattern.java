package frc.lib.hardware.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.hardware.lighting.LEDStrip;
import java.util.Random;
import java.util.stream.IntStream;

/**
 * A space pattern that can be applied to an LED strip.
 * This pattern will create a space-like pattern on the LED strip.
 * @see frc.lib.hardware.lighting.LEDStrip
 */
public class SpacePattern implements LEDPattern {

    private final int seed = 4481;
    private final int maxk = 2;
    private final int n = 123;
    private final Random rng = new Random(seed);
    private final int[] numbers = IntStream.rangeClosed(1, maxk).toArray();
    private final double[][] amplitudesList = new double[maxk][];
    private final double[][] shiftsList = new double[maxk][];

    /**
     * Creates a new SpacePattern with the default seed.
     */
    public SpacePattern (){
        for (int i = 0; i < maxk; i++) {
            amplitudesList[i] = periodicFuncRand(rng, 4, 0.3, n);
            shiftsList[i] = periodicFuncRand(rng, 4, Math.PI / 4, n);
        }
    }

    @Override
    public void updateBuffer(AddressableLEDBuffer buffer, LEDStrip strip) {

        // Update with 25 fps
        double time = Timer.getFPGATimestamp();
        int t = ((int) Math.round(time * 25)) % n;

        int ledLength = strip.getLength();

        double[] hue = IntStream.range(0, ledLength).mapToDouble(i ->
                periodicFunc(numbers,
                        IntStream.range(0, maxk).mapToDouble(j -> amplitudesList[j][t]).toArray(),
                        IntStream.range(0, maxk).mapToDouble(k -> shiftsList[k][t]).toArray(),
                        i / (double) ledLength)
        ).toArray();

        for (int i = 0; i < ledLength; i++) {
            buffer.setHSV(i, ((int) (hue[i] * 180)) % 180, 255, 255);
        }
    }


    @Override
    public String getName() {
        return "SpacePattern";
    }

    private static double periodicFunc(int[] numbers, double[] amplitudes, double[] shifts, double t) {
        double sum = 0;
        for (int i = 0; i < numbers.length; i++) {
            sum += amplitudes[i] * Math.cos(numbers[i] * t * 2 * Math.PI + shifts[i]);
        }
        return sum;
    }

    private static double[] periodicFuncRand(Random rng, int maxk, double scale, int n) {
        int[] numbers = IntStream.rangeClosed(1, maxk).toArray();
//        double[] amplitudes = rng.doubles(maxk, -scale, scale).toArray();
        double[] amplitudes = rng.doubles(maxk, -scale, scale).sorted().map(d -> -d).toArray();
        double[] shifts = rng.doubles(maxk, -Math.PI, Math.PI).toArray();
        return IntStream.range(0, n).mapToDouble(i -> periodicFunc(numbers, amplitudes, shifts, i / (double) n)).toArray();
    }
}
