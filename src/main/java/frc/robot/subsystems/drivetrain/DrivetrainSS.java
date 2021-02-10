package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.Pigeon;
import frc.robot.components.SwerveModule;
import frc.robot.constants.RobotConstants.DrivetrainConstants;
import frc.robot.subsystems.TestableSubsystem;

/**
 * Represents a swerve drive style drivetrain.
 */
public class DrivetrainSS extends SubsystemBase implements TestableSubsystem {
	private final Pigeon gyro;
	private final SwerveDriveOdometry odometry;
	private DrivetrainConstants constants;
	private SwerveDriveKinematics kinematics;
	private SwerveModule[] modules;


	public DrivetrainSS(DrivetrainConstants constants) {
		this.constants = constants;
		initSwerve();
		gyro = constants.canDrivetrainMap.gyro;
		odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());
	}

	/**
	 * Drives the system with the given speeds
	 *
	 * @param speeds the ChassisSpeeds object representing the wanted speeds
	 */
	public void speedDrive(ChassisSpeeds speeds) {
		SwerveModuleState[] states =
				kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.normalizeWheelSpeeds(states, constants.MAX_SPEED);
		setStates(states);
	}

	/**
	 * Drives the robot with the given speeds and angle relative to the field,
	 * and not relative to the robot.
	 *
	 * @param speeds the field relative speeds
	 */
	public void fieldSpeedDrive(ChassisSpeeds speeds) {
		speedDrive(
				ChassisSpeeds.fromFieldRelativeSpeeds(
						speeds.vxMetersPerSecond,
						speeds.vyMetersPerSecond,
						speeds.omegaRadiansPerSecond,
						Rotation2d.fromDegrees(getAngle())
				)
		);
	}

	/**
	 * Drives the system by the given power.
	 *
	 * @param x   x power, between -1 and 1
	 * @param y   y power, between -1 and 1
	 * @param rot rotation power, between -1 and 1
	 */
	public void powerDrive(double x, double y, double rot) {
		x *= constants.MAX_SPEED;
		y *= constants.MAX_SPEED;
		rot *= constants.MAX_ROT_SPEED;
		speedDrive(new ChassisSpeeds(x, y, rot));
	}

	/**
	 * Drives the robot with the given power relative to the field,
	 * and not relative to the robot.
	 *
	 * @param x   field x power, between -1 and 1
	 * @param y   field y power, between -1 and 1
	 * @param rot rotation power, between -1 and 1
	 */
	public void fieldPowerDrive(double x, double y, double rot) {
		x *= constants.MAX_SPEED;
		y *= constants.MAX_SPEED;
		rot *= constants.MAX_ROT_SPEED;
		fieldSpeedDrive(new ChassisSpeeds(x, y, rot));
	}


	/**
	 * Returns the wanted states of the swerve modules as an array
	 *
	 * @return the modules' states
	 */
	public SwerveModuleState[] getStates() {
		SwerveModuleState[] states = new SwerveModuleState[modules.length];
		for (int i = 0; i < modules.length; i++) {
			states[i] = modules[i].getState();
		}
		return states;
	}


	/**
	 * Sets the swerve modules' wanted states
	 *
	 * @param states the modules' states
	 */
	public void setStates(SwerveModuleState[] states) {
		if (states.length != modules.length)
			return;
		for (int i = 0; i < states.length; i++) {
			modules[i].setState(states[i]);
		}
	}


	/**
	 * Returns the angle of the system in degrees
	 *
	 * @return the angle of the system
	 */
	public double getAngle() {
		return gyro.getAngle();
	}


	/**
	 * Drives with the given power. x, y, and rotation.
	 *
	 * @param power the power to give to the system
	 */
	@Override
	public void move(double power) {
		powerDrive(power, power, power);
	}


	/**
	 * Returns an array of the system's sensor values
	 *
	 * @return the sensor values
	 */
	@Override
	public double[] getValues() {
		double[] values = new double[modules.length * 2];
		for (int i = 0; i < modules.length; i++) {
			values[i * 2] = modules[i].getSpeedMotorTicks();
			values[i * 2 + 1] = modules[i].getAngle();
		}
		return values;
	}

	/**
	 * Returns the pose of the system as calculated by the odometry
	 *
	 * @return the pose of the system
	 */
	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	/**
	 * @return the x position on the field as calculated by the odometry
	 */
	public double getX() {
		return getPose().getX();
	}

	/**
	 * @return the y position on the field as calculated by the odometry
	 */
	public double getY() {
		return getPose().getY();
	}


	@Override
	public void periodic() {
		updateOdometry();
		for (SwerveModule module : modules)
			module.periodic();
	}


	private void updateOdometry() {
		odometry.update(gyro.getRotation2d(), getStates());
	}


	private void initSwerve() {
		kinematics = new SwerveDriveKinematics(
				constants.FRONT_RIGHT_LOCATION.getTranslation(),
				constants.FRONT_LEFT_LOCATION.getTranslation(),
				constants.REAR_RIGHT_LOCATION.getTranslation(),
				constants.REAR_LEFT_LOCATION.getTranslation()
		);
		modules = new SwerveModule[]{
				constants.canDrivetrainMap.FRONT_RIGHT,
				constants.canDrivetrainMap.FRONT_LEFT,
				constants.canDrivetrainMap.REAR_RIGHT,
				constants.canDrivetrainMap.REAR_LEFT
		};
	}

}
