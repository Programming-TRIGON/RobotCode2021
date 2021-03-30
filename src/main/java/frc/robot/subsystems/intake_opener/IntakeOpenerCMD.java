package frc.robot.subsystems.intake_opener;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.IntakeOpenerConstants;

import java.util.function.DoubleSupplier;

public class IntakeOpenerCMD extends CommandBase {
    private final IntakeOpenerSS intakeOpenerSS;
    private final IntakeOpenerConstants constants;
    private final DoubleSupplier power;

    public IntakeOpenerCMD(IntakeOpenerSS intakeOpenerSS, IntakeOpenerConstants constants, DoubleSupplier power) {
        this.intakeOpenerSS = intakeOpenerSS;
        this.constants = constants;
        this.power = power;
        addRequirements(this.intakeOpenerSS);
    }

    public IntakeOpenerCMD(IntakeOpenerSS intakeOpenerSS, IntakeOpenerConstants constants) {
        this(intakeOpenerSS, constants, () -> constants.DEFAULT_OPEN_POWER);
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
