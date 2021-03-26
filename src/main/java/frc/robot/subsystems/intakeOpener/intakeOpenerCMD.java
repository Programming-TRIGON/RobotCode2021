package frc.robot.subsystems.intakeOpener;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.IntakeConstants;

import java.util.function.DoubleSupplier;

public class intakeOpenerCMD extends CommandBase {
    private final IntakeOpenerSS intakeOpenerSS;
    private final IntakeConstants constants;
    private final DoubleSupplier power;

    public intakeOpenerCMD(IntakeOpenerSS intakeOpenerSS, IntakeConstants constants, DoubleSupplier power) {
        this.intakeOpenerSS = intakeOpenerSS;
        this.constants = constants;
        this.power = power;
        addRequirements(this.intakeOpenerSS);
    }

    public intakeOpenerCMD(IntakeOpenerSS intakeOpenerSS, IntakeConstants constants) {
        this(intakeOpenerSS, constants, () -> constants.DEFAULT_MOTOR_POWER);
    }

    @Override
    public void execute() {
        intakeOpenerSS.moveWithSafety(power.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return power.getAsDouble() < 0 && intakeOpenerSS.isClosed()
                || power.getAsDouble() > 0 && intakeOpenerSS.isOpen()
                || power.getAsDouble() == 0;
    }

    @Override
    public void end(boolean interrupted) {
        intakeOpenerSS.stopMoving();
    }
}
