package frc.robot.subsystems.led;

/**
 * This class stores the color and amount of times for the LEDs to blink for a given function.
 */
public class LedBlinkColor {

    private LedColor color;
    private int blinkAmount;

    public LedBlinkColor(LedColor color, int blinkAmount) {
        this.color = color;
        this.blinkAmount = blinkAmount;
    }

    public LedColor getColor() {
        return color;
    }

    public int getBlinkAmount() {
        return blinkAmount;
    }
}
