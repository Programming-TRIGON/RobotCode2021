package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.GenericCalibrateKF;
import frc.robot.commands.RunWhenDisabledCommand;
import frc.robot.commands.TurnAndPositionToTargetCMD;
import frc.robot.commands.TurnToTargetCMD;
import frc.robot.commands.command_groups.CollectCMDGP;
import frc.robot.commands.command_groups.ShootCMDGP;
import frc.robot.commands.command_groups.ShootWithPitcherCMDGP;
import frc.robot.constants.RobotConstants;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.TrigonSwerveControllerCMDGP;
import frc.robot.subsystems.drivetrain.SupplierFieldDriveCMD;
import frc.robot.subsystems.intake_opener.IntakeOpenerCMD;
import frc.robot.subsystems.loader.LoaderCMD;
import frc.robot.subsystems.shooter.CalibrateShooterKfCMD;
import frc.robot.subsystems.shooter.ShooterCMD;
import frc.robot.vision.Target;
import frc.robot.vision.limelights.PitcherLimelight;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Contains all the commands used by the driver, operator, or tester. This
 * commands should be called in the RobotContainer either by the controllers or
 * by the Smartdashboard.
 */
public class CommandContainer {

	// CMDGP
	public final ShootCMDGP SHOOT_CMDGP;
	public final ShootWithPitcherCMDGP SHOOT_WITH_PITCHER_CMDGP;
	public final CollectCMDGP COLLECT_CMDGP;

	// Calibration
	public final CalibrateShooterKfCMD CALIBRATE_SHOOTER_KF_CMD;
	public final GenericCalibrateKF CALIBRATE_LOADER_KF_CMD;

	// Shooter
	public final ShooterCMD SHOOTER_CMD;

	// Drivetrain
	public final SupplierFieldDriveCMD SUPPLIER_FIELD_DRIVE_CMD;
	public final TurnToTargetCMD TURN_TO_TARGET_CMD;
	public final TurnAndPositionToTargetCMD TURN_AND_POSITION_TO_TARGET_CMD;
	public final RunWhenDisabledCommand RESET_DIRECTION;
	public final InstantCommand CHANGE_DRIVE_ROTATION;
	public final InstantCommand TOGGLE_DRIVETRAIN_MOTORS_NEUTRAL_MODE_CMD;
	public final TrigonSwerveControllerCMDGP MOTION_TEST;

	// Intake
	public final IntakeOpenerCMD OPEN_INTAKE_CMD;
	public final IntakeOpenerCMD CLOSE_INTAKE_CMD;

	// Pitcher
	public final InstantCommand TOGGLE_PITCHER;
	public final InstantCommand OPEN_PITCHER;
	public final InstantCommand CLOSE_PITCHER;

	// Loader
	public final LoaderCMD LOADER_CMD;

	public CommandContainer(SubsystemContainer subsystemContainer, RobotConstants robotConstants,
			PitcherLimelight limelight, GenericHID driverController) {
		// CMDGP
		SHOOT_CMDGP = new ShootCMDGP(subsystemContainer, robotConstants, limelight);
		SHOOT_WITH_PITCHER_CMDGP = new ShootWithPitcherCMDGP(subsystemContainer, robotConstants, limelight);
		COLLECT_CMDGP = new CollectCMDGP(subsystemContainer, robotConstants);

		// Testing
		CALIBRATE_SHOOTER_KF_CMD = new CalibrateShooterKfCMD(subsystemContainer.SHOOTER_SS,
				robotConstants.shooterConstants);
		CALIBRATE_LOADER_KF_CMD = new GenericCalibrateKF(subsystemContainer.LOADER_SS,
				robotConstants.loaderConstants.FEEDFORWARD_CONSTANTS);

		// Shooter
		SmartDashboard.putNumber("Shooter/Desired Velocity", 0);
		SHOOTER_CMD = new ShooterCMD(subsystemContainer.SHOOTER_SS, null, robotConstants.shooterConstants,
				() -> SmartDashboard.getNumber("Shooter/Desired Velocity", 0));

		// Drivetrain
		SUPPLIER_FIELD_DRIVE_CMD = new SupplierFieldDriveCMD(subsystemContainer.DRIVETRAIN_SS,
				robotConstants.drivetrainConstants,
				() -> Math.signum(driverController.getX(Hand.kRight)) * Math.pow(driverController.getX(Hand.kRight), 2) / 1.5,
				() -> Math.signum(driverController.getY(Hand.kRight)) * Math.pow(driverController.getY(Hand.kRight), 2) / 1.5,
				() -> Math.signum(driverController.getX(Hand.kLeft)) * Math.pow(driverController.getX(Hand.kLeft), 2) / 1.5);
		TURN_TO_TARGET_CMD = new TurnToTargetCMD(subsystemContainer.DRIVETRAIN_SS, limelight,
				robotConstants.visionConstants, Target.PowerPort);
		TURN_AND_POSITION_TO_TARGET_CMD = new TurnAndPositionToTargetCMD(subsystemContainer.DRIVETRAIN_SS, limelight,
				robotConstants.visionConstants, Target.PowerPort);
		RESET_DIRECTION = new RunWhenDisabledCommand(() -> {
			subsystemContainer.DRIVETRAIN_SS.resetGyro();
			subsystemContainer.DRIVETRAIN_SS.resetOdometry(new Pose2d());
		});
		RESET_DIRECTION.addRequirements(subsystemContainer.DRIVETRAIN_SS);
		CHANGE_DRIVE_ROTATION = new InstantCommand(() -> {
			subsystemContainer.DRIVETRAIN_SS.setAngle(subsystemContainer.DRIVETRAIN_SS.getAngle() + 90);
			subsystemContainer.DRIVETRAIN_SS.resetOdometry(
					new Pose2d(0, 0, Rotation2d.fromDegrees(subsystemContainer.DRIVETRAIN_SS.getAngle())));
		});
		CHANGE_DRIVE_ROTATION.addRequirements(subsystemContainer.DRIVETRAIN_SS);
		TOGGLE_DRIVETRAIN_MOTORS_NEUTRAL_MODE_CMD = new InstantCommand(subsystemContainer.DRIVETRAIN_SS::toggleMotorsNeutralMode);
		MOTION_TEST = new TrigonSwerveControllerCMDGP(subsystemContainer.DRIVETRAIN_SS,
				robotConstants.motionProfilingConstants, AutoPath.Test);

		// Intake
		OPEN_INTAKE_CMD = new IntakeOpenerCMD(subsystemContainer.INTAKE_OPENER_SS,
				robotConstants.intakeOpenerConstants, true);
		CLOSE_INTAKE_CMD = new IntakeOpenerCMD(subsystemContainer.INTAKE_OPENER_SS,
		robotConstants.intakeOpenerConstants, true);

		// Pitcher
		TOGGLE_PITCHER = new InstantCommand(subsystemContainer.PITCHER_SS::toggleSolenoid,
				subsystemContainer.PITCHER_SS);
		OPEN_PITCHER = new InstantCommand(() -> subsystemContainer.PITCHER_SS.setSolenoidState(true));
		CLOSE_PITCHER = new InstantCommand(() -> subsystemContainer.PITCHER_SS.setSolenoidState(false));

		// Loader
		LOADER_CMD = new LoaderCMD(subsystemContainer.LOADER_SS, robotConstants.loaderConstants,
				robotConstants.loaderConstants.DEFAULT_SHOOTING_VELOCITY);
	}

}
