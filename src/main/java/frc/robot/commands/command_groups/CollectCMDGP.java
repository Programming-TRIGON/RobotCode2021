package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsytemContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.intake.IntakeCMD;
import frc.robot.subsystems.intake_opener.IntakeOpenerCMD;
import frc.robot.subsystems.loader.LoaderCMD;
import frc.robot.subsystems.spinner.SpinnerCMD;

public class CollectCMDGP extends SequentialCommandGroup {
    private final SubsytemContainer subsystems;
    private final RobotConstants constants;

    public CollectCMDGP(SubsytemContainer subsystems, RobotConstants constants) {
        this.subsystems = subsystems;
        this.constants = constants;

        addRequirements(subsystems.INTAKE_OPENER_SS, subsystems.INTAKE_SS, subsystems.SPINNER_SS, subsystems.LOADER_SS);

        addCommandsToGroup();
    }

    private void addCommandsToGroup() {
        addCommands(new IntakeOpenerCMD(subsystems.INTAKE_OPENER_SS, constants.intakeOpenerConstants),
                new ParallelCommandGroup(
                        new LoaderCMD(subsystems.LOADER_SS, constants.loaderConstants,
                                constants.loaderConstants.DEFAULT_MIXING_VELOCITY),
                        new SpinnerCMD(subsystems.SPINNER_SS, constants.spinnerConstants),
                        new IntakeCMD(subsystems.INTAKE_SS, subsystems.LED_SS, constants.intakeConstants)));
    }
}