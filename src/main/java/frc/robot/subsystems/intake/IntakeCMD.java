package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.led.LedColor;
import frc.robot.subsystems.led.LedSS;

public class IntakeCMD extends CommandBase {
    private IntakeSS intakeSS;
    private RobotConstants.IntakeConstants constants;
    private LedSS ledSS;
    private DoubleSupplier power;
    private double lastStall;
    private double reverseMotorStartTime;

    public IntakeCMD(IntakeSS intakeSS, RobotConstants.IntakeConstants constants, LedSS ledSS, DoubleSupplier power) {
        this.intakeSS = intakeSS;
        this.constants = constants;
        this.ledSS = ledSS;
        this.power = power;
        addRequirements(intakeSS);
    }

    public IntakeCMD(IntakeSS intakeSS, RobotConstants.IntakeConstants constants, LedSS ledSS, double power) {
        this(intakeSS, constants, ledSS, () -> power);
    }

    public IntakeCMD(IntakeSS intakeSS, RobotConstants.IntakeConstants constants, LedSS ledSS) {
        this(intakeSS, constants, ledSS, () -> constants.DEFAULT_MOTOR_POWER);
    }

    @Override
    public void initialize() {
        lastStall = Timer.getFPGATimestamp();
        reverseMotorStartTime = 0;
    }

    /*
     * Checks if motor is stalling as a result of a ball stuck in the intake
     */
    @Override
    public void execute() {
        if (Timer.getFPGATimestamp() - reverseMotorStartTime < constants.STALL_CHECK_DELAY) {
            ledSS.blinkColor(LedColor.Yellow, constants.STALL_BLINK_AMOUNT);
            intakeSS.move(-power.getAsDouble());
        } else {
            if (!intakeSS.isStalled())
                intakeSS.move(power.getAsDouble());
            else {
                reverseMotorStartTime = Timer.getFPGATimestamp();
                ledSS.blinkColor(LedColor.Yellow, constants.STALL_BLINK_AMOUNT);
                intakeSS.move(-power.getAsDouble());
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSS.stopMoving();
    }
}
