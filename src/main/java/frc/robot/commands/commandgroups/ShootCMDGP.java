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

public class ShootCMDGP extends SequentialCommandGroup {
    private final RobotConstants constants;
    private final SubsytemContainer subsystems;
    private final PitcherLimelight limelight;
    private final ShooterCMD shootCMD;
    private ParallelCommandGroup shootParallelGroup;

    public ShootCMDGP(SubsytemContainer subsystems, RobotConstants constants, PitcherLimelight limelight) {
        this.subsystems = subsystems;
        this.constants = constants;
        this.limelight = limelight;

        shootCMD = new ShooterCMD(subsystems.SHOOTER_SS, subsystems.LED_SS, constants.shooterConstants, limelight);

        addCommandsToGroup();
    }

    private void addShootParallelGroup() {
        addCommands(
                new ParallelCommandGroup(
                        new PitcherCMD(subsystems.PITCHER_SS, subsystems.LED_SS, constants.pitcherConstants, limelight),
                        shootCMD,
                        new SequentialCommandGroup(
                                new GenericTurnToTargetCMD(limelight, constants.visionConstants,
                                        Target.PowerPort, subsystems.DRIVETRAIN_SS),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(shootCMD::atSetpoint),
                                        new ParallelCommandGroup(
                                                new LoaderCMD(subsystems.LOADER_SS, constants.loaderConstants, constants.loaderConstants.DEFAULT_SHOOTING_VELOCITY),
                                                new SpinnerCMD(subsystems.SPINNER_SS, constants.spinnerConstants)
                                        ))
                        )
                )
        );
    }

    private void addCommandsToGroup() {
        addCommands(
                new ConditionalCommand(
                        new InstantCommand(this::addShootParallelGroup),
                        new SequentialCommandGroup(
                                new InstantCommand(subsystems.PITCHER_SS::toggleSolenoid, subsystems.PITCHER_SS),
                                new ConditionalCommand(
                                        new InstantCommand(this::addShootParallelGroup),
                                        new InstantCommand(),
                                        limelight::getTv
                                )
                        ),
                        limelight::getTv
                )
        );
    }


}