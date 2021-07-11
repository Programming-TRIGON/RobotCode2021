package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.SubsystemContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.led.BlinkAndLogCMD;
import frc.robot.subsystems.pitcher.PitcherCMD;
import frc.robot.subsystems.shooter.ShooterCMD;
import frc.robot.vision.limelights.PitcherLimelight;

import java.util.function.DoubleSupplier;

public class ShootWithPitcherCMDGP extends SequentialCommandGroup {
        private final RobotConstants constants;
        private final SubsystemContainer subsystems;
        private final PitcherLimelight limelight;
        private final DoubleSupplier desiredVelocity;

        public ShootWithPitcherCMDGP(SubsystemContainer subsystems, RobotConstants constants,
                        PitcherLimelight limelight, DoubleSupplier desiredVelocity) {
                this.subsystems = subsystems;
                this.constants = constants;
                this.limelight = limelight;
                this.desiredVelocity = desiredVelocity;

                if (subsystems.LED_SS != null)
                        addRequirements(subsystems.DRIVETRAIN_SS, subsystems.PITCHER_SS, subsystems.LED_SS,
                                        subsystems.SHOOTER_SS, subsystems.LOADER_SS, subsystems.SPINNER_SS);
                else
                        addRequirements(subsystems.DRIVETRAIN_SS, subsystems.PITCHER_SS, subsystems.SHOOTER_SS,
                                        subsystems.LOADER_SS, subsystems.SPINNER_SS);
                addCommandsToGroup();
        }

        public ShootWithPitcherCMDGP(SubsystemContainer subsystems, RobotConstants constants,
                                     PitcherLimelight limelight) {
                this(subsystems, constants, limelight, limelight::calculateDesiredShooterVelocity);
        }

        private void addCommandsToGroup() {
                addCommands(new ConditionalCommand(
                                new SequentialCommandGroup(
                                                new PitcherCMD(subsystems.PITCHER_SS, subsystems.LED_SS,
                                                                constants.pitcherConstants, limelight),
                                                new ShootCMDGP(subsystems, constants, limelight, desiredVelocity)),
                                new SequentialCommandGroup(
                                                new InstantCommand(subsystems.PITCHER_SS::toggleSolenoid,
                                                                subsystems.PITCHER_SS),
                                                new WaitCommand(0.2),
                                                new ConditionalCommand(
                                                                new ShootCMDGP(subsystems, constants, limelight, desiredVelocity),
                                                                new BlinkAndLogCMD(subsystems.LED_SS,
                                                                                "ShootCMDGP: NO TARGET FOUND! try repositioning",
                                                                                constants.ledConstants.COLOR_MAP.NO_TARGET),
                                                                limelight::hasTarget)),
                                limelight::hasTarget));
        }
}
