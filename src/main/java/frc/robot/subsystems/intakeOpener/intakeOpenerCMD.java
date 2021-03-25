package frc.robot.subsystems.intakeOpener;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.IntakeConstants;
import frc.robot.subsystems.intakeOpener.IntakeOpenerSS;

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
        if (intakeOpenerSS.isOpen())
            intakeOpenerSS.moveWithSafety(power.getAsDouble());
        else
            intakeOpenerSS.moveWithSafety(-power.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return intakeOpenerSS.isOpen() || intakeOpenerSS.isClosed();
    }

    @Override
    public void end(boolean interrupted) {
        intakeOpenerSS.stopMoving();
    }
}
