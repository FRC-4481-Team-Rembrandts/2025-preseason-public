package frc.lib.hardware.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.lib.hardware.lighting.LEDStrip;

/**
 * A pattern that can be applied to an LED strip.
 * @see frc.lib.hardware.lighting.LEDStrip
 */
public interface LEDPattern {

    /**
     * Updates the LED buffer with the pattern.
     * @param buffer The LED buffer to update.
     * @param strip The strip to update the buffer with.
     */
    void updateBuffer(AddressableLEDBuffer buffer, LEDStrip strip);

    /**
     * Gets the name of the pattern.
     * @return The name of the pattern.
     */
    String getName();
}
