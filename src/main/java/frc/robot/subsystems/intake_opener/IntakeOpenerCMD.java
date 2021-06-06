package frc.robot.subsystems.intake_opener;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.RobotConstants.IntakeOpenerConstants;


public class IntakeOpenerCMD extends InstantCommand {
    private final IntakeOpenerSS intakeOpenerSS;
    private final IntakeOpenerConstants constants;
    private final boolean isOpen;

    public IntakeOpenerCMD(boolean isOpen,IntakeOpenerSS intakeOpenerSS, IntakeOpenerConstants constants) {
        this.intakeOpenerSS = intakeOpenerSS;
        this.constants = constants;
        this.isOpen=isOpen;

        addRequirements(intakeOpenerSS);
    }


    @Override
    public void execute() {
        intakeOpenerSS.setSolenoidState(isOpen);
    }

}
