package frc.robot.utilities;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class TrigonXboxController extends XboxController {
    private static final double kIntermittentRumbleTime = 0.15;
    private final JoystickButton buttonA;
    private final JoystickButton buttonB;
    private final JoystickButton buttonX;
    private final JoystickButton buttonY;
    private final JoystickButton leftBumper;
    private final JoystickButton rightBumper;
    private final JoystickButton leftStickButton;
    private final JoystickButton rightStickButton;
    private final JoystickButton backButton;
    private final JoystickButton startButton;
    private final Notifier notifier;
    private int rumbleAmount;

    public TrigonXboxController(int port) {
        super(port);
        buttonA = new JoystickButton(this, Button.kA.value);
        buttonB = new JoystickButton(this, Button.kB.value);
        buttonX = new JoystickButton(this, Button.kX.value);
        buttonY = new JoystickButton(this, Button.kY.value);
        leftBumper = new JoystickButton(this, Button.kBumperLeft.value);
        rightBumper = new JoystickButton(this, Button.kBumperRight.value);
        leftStickButton = new JoystickButton(this, Button.kStickLeft.value);
        rightStickButton = new JoystickButton(this, Button.kStickRight.value);

        backButton = new JoystickButton(this, Button.kBack.value);
        startButton = new JoystickButton(this, Button.kStart.value);
        rumbleAmount = -1;
        notifier = new Notifier(this::notifierPeriodic);
    }

    public JoystickButton getButtonA() {
        return buttonA;
    }

    public JoystickButton getButtonB() {
        return buttonB;
    }

    public JoystickButton getButtonX() {
        return buttonX;
    }

    public JoystickButton getButtonY() {
        return buttonY;
    }

    public JoystickButton getLeftBumper() {
        return leftBumper;
    }

    public JoystickButton getRightBumper() {
        return rightBumper;
    }

    public JoystickButton getLeftStickButton() {
        return leftStickButton;
    }

    public JoystickButton getRightStickButton() {
        return rightStickButton;
    }

    public JoystickButton getBackXboxButton() {
        return backButton;
    }

    public JoystickButton getStartXboxButton() {
        return startButton;
    }

    /**
     * This method returns the positive trigger minus the negative trigger. This is
     * used the for controlling the speed on the Y axis of the robot.
     *
     * @param positiveHand The hand that is positive, the other will be negative.
     * @return positive trigger value - other trigger value.
     */
    public double getDeltaTriggers(Hand positiveHand) {

        if (positiveHand == Hand.kRight)
            return getTriggerAxis(Hand.kRight) - getTriggerAxis(Hand.kLeft);
        else
            return getTriggerAxis(Hand.kLeft) - getTriggerAxis(Hand.kRight);
    }

    /**
     * Right trigger value - left trigger value.
     *
     * @return The difference between the left and right triggers.
     */
    public double getDeltaTriggers()
    { if (getTriggerAxis(Hand.kRight) - getTriggerAxis(Hand.kLeft) > 0)
        return (getTriggerAxis(Hand.kRight) - getTriggerAxis(Hand.kLeft)) * 0.95;
        else return getTriggerAxis(Hand.kRight) - getTriggerAxis(Hand.kLeft);
    }

    /**
     * Set the rumble output for the HID. this method affects both motors.
     *
     * @param power The normalized value (0 to 1) to set the rumble to
     */
    public void setRumble(double power) {
        setRumble(RumbleType.kLeftRumble, power);
        setRumble(RumbleType.kRightRumble, power);
    }

    /**
     * Rumbles the controller the desired amount of times.
     *
     * @param quantity the number of times to rumble.
     */
    public void intermittentRumble(int quantity) {
        rumbleAmount = quantity * 2 - 1;
        notifier.startPeriodic(kIntermittentRumbleTime);
    }

    @Override
    public double getY(Hand hand) {
        if (Math.abs(super.getY(hand)) < 0.1) return 0;
        return -super.getY(hand);
    }

    @Override
    public double getX(Hand hand) {
        if (Math.abs(super.getX(hand)) < 0.1) return 0;
        return super.getX(hand);
    }

    public void notifierPeriodic() {
        if (rumbleAmount == 0) {
            notifier.stop();
        }
        if (rumbleAmount >= 0) {
            if (rumbleAmount % 2 == 1)
                setRumble(1);
            else
                setRumble(0);
            rumbleAmount--;
        }
    }
}
