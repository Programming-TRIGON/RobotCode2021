package frc.robot.commands.commandgroups;


import edu.wpi.first.wpilibj2.command.*;
import frc.robot.SubsytemContainer;
import frc.robot.commands.GenericTurnToTargetCMD;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.loader.LoaderCMD;
import frc.robot.subsystems.pitcher.PitcherCMD;
import frc.robot.subsystems.shooter.ShooterCMD;
import frc.robot.subsystems.spinner.SpinnerCMD;
import frc.robot.vision.Target;
import frc.robot.vision.limelights.PitcherLimelight;

public class shootCMDGP extends SequentialCommandGroup {
    private final RobotConstants constants;
    private final SubsytemContainer subsystems;
    private final PitcherLimelight limelight;
    private final ShooterCMD shootCMD;
    private ParallelCommandGroup shootParallelGroup;

    public shootCMDGP(SubsytemContainer subsystems, RobotConstants constants, PitcherLimelight limelight) {
        this.subsystems = subsystems;
        this.constants = constants;
        this.limelight = limelight;

        shootCMD = new ShooterCMD(subsystems.SHOOTERSS, subsystems.LEDSS, constants.shooterConstants, limelight);

        addCommandsToGroup();
    }

    private void addCommandsToGroup() {
        addCommands(
                new ConditionalCommand(
                        new InstantCommand(this::addCommandsToGroup),
                        new SequentialCommandGroup(
                                new InstantCommand(subsystems.PITCHERSS::toggleSolenoid, subsystems.PITCHERSS),
                                new ConditionalCommand(
                                        new InstantCommand(this::addCommandsToGroup),
                                        new InstantCommand(),
                                        limelight::getTv
                                )
                        ),
                        limelight::getTv
                )
        );
    }

    private void addShootParallelGroup() {
        addCommands(
                new ParallelCommandGroup(
                        new PitcherCMD(subsystems.PITCHERSS, subsystems.LEDSS, constants.pitcherConstants, limelight),
                        shootCMD,
                        new SequentialCommandGroup(
                                new GenericTurnToTargetCMD(limelight, constants.visionConstants,
                                        Target.PowerPort, subsystems.DRIVETRAINSS),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(shootCMD::atSetpoint),
                                        new ParallelCommandGroup(
                                                new LoaderCMD(subsystems.LOADERSS, constants.loaderConstants),
                                                new SpinnerCMD(subsystems.SPINNERSS, constants.spinnerConstants)
                                        ))
                        )
                )
        );
    }
}