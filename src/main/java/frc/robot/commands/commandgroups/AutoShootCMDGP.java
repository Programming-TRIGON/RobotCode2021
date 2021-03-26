package frc.robot.commands.commandgroups;


import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SubsytemContainer;
import frc.robot.commands.GenericTurnToTargetCMD;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.loader.LoaderCMD;
import frc.robot.subsystems.pitcher.PitcherCMD;
import frc.robot.subsystems.shooter.ShooterCMD;
import frc.robot.subsystems.spinner.SpinnerCMD;
import frc.robot.vision.Target;
import frc.robot.vision.limelights.PitcherLimelight;

public class AutoShootCMDGP extends SequentialCommandGroup {
    private final RobotConstants constants;
    private final SubsytemContainer subsystems;
    private final PitcherLimelight limelight;
    private final ShooterCMD shootCMD;

    public AutoShootCMDGP(RobotConstants constants, PitcherLimelight limelight, SubsytemContainer subsystems) {
        this.constants = constants;
        this.subsystems = subsystems;
        this.limelight = limelight;

        shootCMD = new ShooterCMD(subsystems.SHOOTERSS, constants.shooterConstants, subsystems.LEDSS, limelight);

        addCommandToGroup();
    }

    private void addCommandToGroup() {
        addCommands(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new PitcherCMD(subsystems.PITCHERSS, subsystems.LEDSS, constants.pitcherConstants, limelight),
                                new GenericTurnToTargetCMD(limelight, constants.visionConstants, Target.PowerPort, subsystems.DRIVETRAINSS),
                                shootCMD,
                                new WaitUntilCommand(shootCMD::atSetpoint),
                                parallel(
                                        new LoaderCMD(subsystems.LOADERSS, constants.loaderConstants),
                                        new SpinnerCMD(subsystems.SPINNERSS, constants.spinnerConstants)
                                )
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(subsystems.PITCHERSS::toggleSolenoid, subsystems.PITCHERSS),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new PitcherCMD(subsystems.PITCHERSS, subsystems.LEDSS, constants.pitcherConstants, limelight),
                                                new GenericTurnToTargetCMD(limelight, constants.visionConstants,
                                                        Target.PowerPort, subsystems.DRIVETRAINSS),
                                                shootCMD,
                                                new WaitUntilCommand(shootCMD::atSetpoint),
                                                parallel(
                                                        new LoaderCMD(subsystems.LOADERSS, constants.loaderConstants),
                                                        new SpinnerCMD(subsystems.SPINNERSS, constants.spinnerConstants)
                                                )
                                        ),
                                        new InstantCommand(),
                                        limelight::getTv
                                )
                        ),
                        limelight::getTv
                )
        );
    }
}