package frc.robot.subsystems.spinner;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.SpinnerConstants;
import frc.robot.utilities.DriverStationLogger;

import java.util.function.DoubleSupplier;

public class SpinnerCMD extends CommandBase {
    private final SpinnerSS spinnerSS;
    private final SpinnerConstants constants;
    private DoubleSupplier power;
    private double output;
    private double initialOutput;
    private double reverseMotorStartTime;
    private boolean stillStalled;

    public SpinnerCMD(SpinnerSS spinnerSS, SpinnerConstants constants, DoubleSupplier power) {
        this.spinnerSS = spinnerSS;
        this.constants = constants;
        this.power = power;
        addRequirements(spinnerSS);
    }

    public SpinnerCMD(SpinnerSS spinnerSS, SpinnerConstants constants) {
        this(spinnerSS, constants, () -> constants.DEFAULT_MOTOR_POWER);
    }

    @Override
    public void initialize() {
        reverseMotorStartTime = 0;
        stillStalled = false;
        output = power.getAsDouble();
        initialOutput = output;
    }

    @Override
    public void execute() {
        // Checks if motor is stalling as a result of a ball stuck in the intake and
        // acts accordingly
        if (!spinnerSS.isStalled() && Timer.getFPGATimestamp() - reverseMotorStartTime >= constants.STALL_CHECK_DELAY) {
            output = initialOutput;
            stillStalled = false;
        } else {
            if (!stillStalled) {
                reverseMotorStartTime = Timer.getFPGATimestamp();
                DriverStationLogger.logToDS("A ball is stuck in the spinner, trying to release it!");
                output = -initialOutput;
            }
            if (Timer.getFPGATimestamp() - reverseMotorStartTime >= constants.STALL_CHECK_DELAY
                    && spinnerSS.isStalled()) {
                output = -output;
                reverseMotorStartTime = Timer.getFPGATimestamp();
            }
            stillStalled = true;
        }
        spinnerSS.move(output);
    }

    @Override
    public void end(boolean interrupted) {
        spinnerSS.stopMoving();
    }
}
