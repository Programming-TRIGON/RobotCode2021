package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.SubsystemContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.led.BlinkAndLogCMD;
import frc.robot.subsystems.pitcher.PitcherCMD;
import frc.robot.subsystems.shooter.ShooterCMD;
import frc.robot.vision.limelights.PitcherLimelight;

public class ShootWithPitcherCMDGP extends SequentialCommandGroup {
        private final RobotConstants constants;
        private final SubsystemContainer subsystems;
        private final PitcherLimelight limelight;

        public ShootWithPitcherCMDGP(SubsystemContainer subsystems, RobotConstants constants,
                        PitcherLimelight limelight) {
                this.subsystems = subsystems;
                this.constants = constants;
                this.limelight = limelight;

                if (subsystems.LED_SS != null)
                        addRequirements(subsystems.DRIVETRAIN_SS, subsystems.PITCHER_SS, subsystems.LED_SS,
                                        subsystems.SHOOTER_SS, subsystems.LOADER_SS, subsystems.SPINNER_SS);
                else
                        addRequirements(subsystems.DRIVETRAIN_SS, subsystems.PITCHER_SS, subsystems.SHOOTER_SS,
                                        subsystems.LOADER_SS, subsystems.SPINNER_SS);
                addCommandsToGroup();
        }

        private void addCommandsToGroup() {
                addCommands(new ConditionalCommand(
                                new SequentialCommandGroup(
                                                new PitcherCMD(subsystems.PITCHER_SS, subsystems.LED_SS,
                                                                constants.pitcherConstants, limelight),
                                                new ShootCMDGP(subsystems, constants, limelight)),
                                new SequentialCommandGroup(
                                                new InstantCommand(subsystems.PITCHER_SS::toggleSolenoid,
                                                                subsystems.PITCHER_SS),
                                                new ConditionalCommand(new SequentialCommandGroup(
                                                                new PitcherCMD(subsystems.PITCHER_SS, subsystems.LED_SS,
                                                                                constants.pitcherConstants, limelight),
                                                                new ShootCMDGP(subsystems, constants, limelight)),
                                                                new BlinkAndLogCMD(subsystems.LED_SS,
                                                                                "ShootCMDGP: NO TARGET FOUND! try repositioning",
                                                                                constants.ledConstants.COLOR_MAP.NO_TARGET),
                                                                limelight::hasTarget)),
                                limelight::hasTarget));
        }
}