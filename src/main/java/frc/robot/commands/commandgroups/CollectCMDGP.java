package frc.robot.commands.commandgroups;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SubsytemContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.intake.IntakeCMD;
import frc.robot.subsystems.intakeOpener.intakeOpenerCMD;
import frc.robot.subsystems.spinner.SpinnerCMD;

import java.util.function.BooleanSupplier;

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
                        new intakeOpenerCMD(subsystems.INTAKEOPENERSS, constants.intakeOpenerConstants),
                        new WaitUntilCommand(subsystems.INTAKEOPENERSS::isOpen),
                        //TODO: add intake opener closing once OI is built
                        new ParallelCommandGroup(
                                new SpinnerCMD(subsystems.SPINNERSS, constants.spinnerConstants),
                                new IntakeCMD(subsystems.INTAKESS, subsystems.LEDSS, constants.intakeConstants)
                        )
                )
        );
    }
}