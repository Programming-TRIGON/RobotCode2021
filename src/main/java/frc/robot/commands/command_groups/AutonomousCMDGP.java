package frc.robot.commands.command_groups;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.SubsystemContainer;
import frc.robot.commands.TurnToTargetCMD;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drivetrain.SupplierDriveCMD;
import frc.robot.subsystems.loader.LoaderCMD;
import frc.robot.subsystems.shooter.ShooterCMD;
import frc.robot.subsystems.spinner.SpinnerCMD;
import frc.robot.vision.Target;
import frc.robot.vision.limelights.PitcherLimelight;

public class AutonomousCMDGP extends SequentialCommandGroup {
    private final SubsystemContainer subsystems;
    private final RobotConstants constants;
    private final PitcherLimelight limelight;

    public AutonomousCMDGP(SubsystemContainer subsystems, RobotConstants constants, PitcherLimelight limelight) {
        this.subsystems = subsystems;
        this.constants = constants;
        this.limelight = limelight;

        addRequirements(subsystems.DRIVETRAIN_SS, subsystems.SHOOTER_SS, subsystems.PITCHER_SS,
                subsystems.LOADER_SS, subsystems.SPINNER_SS);

        ShooterCMD shootCMD = new ShooterCMD(subsystems.SHOOTER_SS, subsystems.LED_SS,
                constants.shooterConstants, limelight, 3);

        addCommands(
                new SequentialCommandGroup(
                        shootCMD,
                        new SequentialCommandGroup(
                                new TurnToTargetCMD(subsystems.DRIVETRAIN_SS, limelight, constants.visionConstants, Target.PowerPort),
                                new WaitUntilCommand(shootCMD::isAtSetpoint),
                                new ParallelCommandGroup(
                                        new LoaderCMD(subsystems.LOADER_SS, constants.loaderConstants,
                                                constants.loaderConstants.DEFAULT_SHOOTING_VELOCITY),
                                        new SpinnerCMD(subsystems.SPINNER_SS, constants.spinnerConstants)
                                )
                        ),
                        new WaitUntilCommand(shootCMD::allBallsShot),
                        new SupplierDriveCMD(subsystems.DRIVETRAIN_SS, () -> 0.0, () -> 0.5, () -> 0.0),
                        new WaitCommand(5)
                )
        );
    }
}