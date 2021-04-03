package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LedSS;
import frc.robot.utilities.DriverStationLogger;
import frc.robot.constants.RobotConstants.IntakeConstants;

public class IntakeCMD extends CommandBase {
    private final IntakeSS intakeSS;
    private final IntakeConstants constants;
    private final LedSS ledSS;
    private final DoubleSupplier power;
    private double reverseMotorStartTime;
    private boolean stillStalled;

    public IntakeCMD(IntakeSS intakeSS, LedSS ledSS, IntakeConstants constants, DoubleSupplier power) {
        this.intakeSS = intakeSS;
        this.constants = constants;
        this.ledSS = ledSS;
        this.power = power;
        addRequirements(intakeSS);
    }

    public IntakeCMD(IntakeSS intakeSS, LedSS ledSS, IntakeConstants constants, double power) {
        this(intakeSS, ledSS, constants, () -> power);
    }

    public IntakeCMD(IntakeSS intakeSS, LedSS ledSS, IntakeConstants constants) {
        this(intakeSS, ledSS, constants, () -> constants.DEFAULT_MOTOR_POWER);
    }

    @Override
    public void initialize() {
        reverseMotorStartTime = 0;
        stillStalled = false;
    }

    @Override
    public void execute() {
        // Checks if motor is stalling as a result of a ball stuck in the intake and acts accordingly
        if (!intakeSS.isStalled() && Timer.getFPGATimestamp() - reverseMotorStartTime >= constants.STALL_CHECK_DELAY) {
            intakeSS.move(power.getAsDouble());
            if(ledSS != null)
                ledSS.setColor(ledSS.getColorMap().INTAKE_ENABLED);
            stillStalled = false;
        }
        else {
            if (!stillStalled) {
                reverseMotorStartTime = Timer.getFPGATimestamp();
                DriverStationLogger.logToDS("A ball is stuck in the intake, trying to release it!");
            }
            intakeSS.move(-power.getAsDouble());
            if(ledSS != null)
                ledSS.blinkColor(ledSS.getColorMap().INTAKE_MOTOR_STALL);
            stillStalled = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(ledSS != null)
            ledSS.turnOffLED();
        intakeSS.stopMoving();
    }
}
