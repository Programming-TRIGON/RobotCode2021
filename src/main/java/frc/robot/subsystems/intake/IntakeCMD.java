package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.IntakeConstants;
import frc.robot.constants.RobotConstants.SpinnerConstants;
import frc.robot.utilities.DriverStationLogger;

import java.util.function.DoubleSupplier;

public class IntakeCMD extends CommandBase {
    private final IntakeSS intakeSS;
    private final IntakeConstants constants;
    private DoubleSupplier power;
    private double output;
    private double initialOutput;
    private double reverseMotorStartTime;
    private boolean stillStalled;
    private boolean stallLogic;

    public IntakeCMD(IntakeSS intakeSS, IntakeConstants constants, DoubleSupplier power, boolean stallLogic) {
        this.intakeSS = intakeSS;
        this.constants = constants;
        this.power = power;
        this.stallLogic = stallLogic;
        addRequirements(intakeSS);
    }

    public IntakeCMD(IntakeSS intakeSS, IntakeConstants constants, DoubleSupplier power) {
        this(intakeSS, constants, power, false);
    }

    public IntakeCMD(IntakeSS intakeSS, IntakeConstants constants) {
        this(intakeSS, constants, () -> constants.DEFAULT_MOTOR_POWER);
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
        // Checks if motor is stalling as a result of a ball stuck in the spinner and
        // acts accordingly
        if (stallLogic) {
            if (!intakeSS.isStalled()
                    && Timer.getFPGATimestamp() - reverseMotorStartTime >= constants.STALL_CHECK_DELAY) {
                output = initialOutput;
                stillStalled = false;
            } else {
                if (!stillStalled) {
                    reverseMotorStartTime = Timer.getFPGATimestamp();
                    DriverStationLogger.logToDS("A ball is stuck in the loader, trying to release it!");
                    output = -initialOutput;
                }
                if (Timer.getFPGATimestamp() - reverseMotorStartTime >= constants.STALL_CHECK_DELAY
                        && intakeSS.isStalled()) {
                    output = -output;
                    reverseMotorStartTime = Timer.getFPGATimestamp();
                }
                stillStalled = true;
            }
            intakeSS.move(output);
        } else
            intakeSS.move(output);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSS.stopMoving();
    }
}
