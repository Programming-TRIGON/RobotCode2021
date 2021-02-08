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

    public IntakeCMD(IntakeSS intakeSS, IntakeConstants constants, LedSS ledSS, DoubleSupplier power) {
        this.intakeSS = intakeSS;
        this.constants = constants;
        this.ledSS = ledSS;
        this.power = power;
        addRequirements(intakeSS);
    }

    public IntakeCMD(IntakeSS intakeSS, IntakeConstants constants, LedSS ledSS, double power) {
        this(intakeSS, constants, ledSS, () -> power);
    }

    public IntakeCMD(IntakeSS intakeSS, IntakeConstants constants, LedSS ledSS) {
        this(intakeSS, constants, ledSS, () -> constants.DEFAULT_MOTOR_POWER);
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
            ledSS.setColor(ledSS.getColorMap().INTAKE_ENABLED);
            stillStalled = false;
        }
        else {
            if (stillStalled) {
                reverseMotorStartTime = Timer.getFPGATimestamp();
                DriverStationLogger.logToDS("A ball is stuck in the intake, trying to release it!");
            }
            intakeSS.move(-power.getAsDouble());
            ledSS.blinkColor(ledSS.getColorMap().INTAKE_MOTOR_STALL);
            stillStalled = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        ledSS.turnOffLED();
        intakeSS.stopMoving();
    }
}
