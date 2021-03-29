package frc.robot.commands.commandgroups;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SubsytemContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.intake.IntakeCMD;
import frc.robot.subsystems.intakeOpener.intakeOpenerCMD;
import frc.robot.subsystems.loader.LoaderCMD;
import frc.robot.subsystems.spinner.SpinnerCMD;

public class CollectCMDGP extends SequentialCommandGroup {
    private final SubsytemContainer subsystems;
    private final RobotConstants constants;

    public CollectCMDGP(SubsytemContainer subsystems, RobotConstants constants) {
        this.subsystems = subsystems;
        this.constants = constants;
    }

    private void addCommandsToGroup() {
        addCommands(
                new SequentialCommandGroup(
                        new intakeOpenerCMD(subsystems.INTAKE_OPENER_SS, constants.intakeOpenerConstants),
                        new WaitUntilCommand(subsystems.INTAKE_OPENER_SS::isOpen),
                        //TODO: add intake opener closing once OI is built
                        new ParallelCommandGroup(
                                new LoaderCMD(subsystems.LOADER_SS, constants.loaderConstants, constants.loaderConstants.DEFAULT_MIXING_VELOCITY),
                                new SpinnerCMD(subsystems.SPINNER_SS, constants.spinnerConstants),
                                new IntakeCMD(subsystems.INTAKE_SS, subsystems.LED_SS, constants.intakeConstants)
                        )
                )
        );
    }
}