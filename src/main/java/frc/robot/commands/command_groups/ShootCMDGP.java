package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.SubsytemContainer;
import frc.robot.commands.turnToTargetCMD;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.led.BlinkAndLogCMD;
import frc.robot.subsystems.loader.LoaderCMD;
import frc.robot.subsystems.pitcher.PitcherCMD;
import frc.robot.subsystems.shooter.ShooterCMD;
import frc.robot.subsystems.spinner.SpinnerCMD;
import frc.robot.vision.Target;
import frc.robot.vision.limelights.PitcherLimelight;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class ShootCMDGP extends SequentialCommandGroup {
        private final RobotConstants constants;
        private final SubsytemContainer subsystems;
        private final PitcherLimelight limelight;
        private final ShooterCMD shootCMD;
        private final GenericHID genericHID;

        public ShootCMDGP(SubsytemContainer subsystems, RobotConstants constants, PitcherLimelight limelight,
                        GenericHID genericHID) {
                this.subsystems = subsystems;
                this.constants = constants;
                this.limelight = limelight;
                this.genericHID = genericHID;

                if (subsystems.LED_SS != null)
                        addRequirements(subsystems.DRIVETRAIN_SS, subsystems.PITCHER_SS, subsystems.LED_SS,
                                        subsystems.SHOOTER_SS, subsystems.LOADER_SS, subsystems.SPINNER_SS);
                else
                        addRequirements(subsystems.DRIVETRAIN_SS, subsystems.PITCHER_SS, subsystems.SHOOTER_SS,
                                        subsystems.LOADER_SS, subsystems.SPINNER_SS);

                shootCMD = new ShooterCMD(subsystems.SHOOTER_SS, subsystems.LED_SS, constants.shooterConstants,
                                limelight);

                addCommandsToGroup();
        }

        private void addCommandsToGroup() {
                addCommands(new ConditionalCommand(new InstantCommand(this::addShootingCommandGroup),
                                new SequentialCommandGroup(
                                                new InstantCommand(subsystems.PITCHER_SS::toggleSolenoid,
                                                                subsystems.PITCHER_SS),
                                                new ConditionalCommand(
                                                                new InstantCommand(this::addShootingCommandGroup),
                                                                new BlinkAndLogCMD(subsystems.LED_SS,
                                                                                "ShootCMDGP: NO TARGET FOUND! try repositioning",
                                                                                constants.ledConstants.COLOR_MAP.NO_TARGET),
                                                                limelight::getTv)),
                                limelight::getTv),
                                new InstantCommand());
        }

        private void addShootingCommandGroup() {
                addCommands(new SequentialCommandGroup(
                                new PitcherCMD(subsystems.PITCHER_SS, subsystems.LED_SS, constants.pitcherConstants,
                                                limelight),
                                new ParallelCommandGroup(shootCMD, new SequentialCommandGroup(
                                                new turnToTargetCMD(subsystems.DRIVETRAIN_SS, limelight,
                                                                constants.visionConstants, Target.PowerPort),
                                                new WaitUntilCommand(shootCMD::atSetpoint),
                                                new ParallelCommandGroup(new LoaderCMD(subsystems.LOADER_SS,
                                                                constants.loaderConstants,
                                                                constants.loaderConstants.DEFAULT_SHOOTING_VELOCITY),
                                                                new SpinnerCMD(subsystems.SPINNER_SS,
                                                                                constants.spinnerConstants))))));
        }

        /**
         * Checks the values of the driving joysticks and if one of them is above a
         * specified threshold. This is done incase the driver desires to continue
         * moving before the robot is finished shooting.
         * 
         * @return if to interrupt the shooterCMDGP.
         */
        @Override
        public boolean isFinished() {
                double threshold = constants.shooterConstants.CANCEL_CMDGP_AXIS_THRESHOLD;
                return Math.abs(genericHID.getX(Hand.kRight)) >= threshold
                                || Math.abs(genericHID.getY(Hand.kRight)) >= threshold
                                || Math.abs(genericHID.getX(Hand.kLeft)) >= threshold;
        }

}