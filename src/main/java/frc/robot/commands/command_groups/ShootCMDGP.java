package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SubsystemContainer;
import frc.robot.commands.TurnToTargetCMD;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.loader.LoaderCMD;
import frc.robot.subsystems.shooter.ShooterCMD;
import frc.robot.subsystems.spinner.SpinnerCMD;
import frc.robot.vision.Target;
import frc.robot.vision.limelights.PitcherLimelight;

public class ShootCMDGP extends ParallelCommandGroup {
        public ShootCMDGP(SubsystemContainer subsystems, RobotConstants constants, PitcherLimelight limelight) {

                ShooterCMD shootCMD = new ShooterCMD(subsystems.SHOOTER_SS, subsystems.LED_SS, constants.shooterConstants, limelight);
                addCommands(
                        shootCMD, 
                        new SequentialCommandGroup(new TurnToTargetCMD(subsystems.DRIVETRAIN_SS,
                                limelight, constants.visionConstants, Target.PowerPort),
                        new WaitUntilCommand(shootCMD::isAtSetpoint),
                        new ParallelCommandGroup(new LoaderCMD(subsystems.LOADER_SS, constants.loaderConstants,
                                constants.loaderConstants.DEFAULT_SHOOTING_VELOCITY),
                        new SpinnerCMD(subsystems.SPINNER_SS, constants.spinnerConstants))));
        }

        public ShootCMDGP(SubsystemContainer subsystems, RobotConstants constants, PitcherLimelight limelight,
                        double desiredVelocity) {

                ShooterCMD shootCMD = new ShooterCMD(subsystems.SHOOTER_SS, subsystems.LED_SS,
                                constants.shooterConstants, () -> desiredVelocity);
                addCommands(
                        shootCMD,
                        new SequentialCommandGroup(new TurnToTargetCMD(subsystems.DRIVETRAIN_SS,
                                limelight, constants.visionConstants, Target.PowerPort),
                        new WaitUntilCommand(shootCMD::isAtSetpoint),
                        new ParallelCommandGroup(
                                new LoaderCMD(subsystems.LOADER_SS, constants.loaderConstants,
                                        constants.loaderConstants.DEFAULT_SHOOTING_VELOCITY),
                                new SpinnerCMD(subsystems.SPINNER_SS, constants.spinnerConstants))));
        }
}
