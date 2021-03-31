package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.Pigeon;
import frc.robot.components.SwerveModule;
import frc.robot.constants.RobotConstants.DrivetrainConstants;
import frc.robot.subsystems.TestableSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Represents a swerve drive style drivetrain.
 */
public class DrivetrainSS extends SubsystemBase implements TestableSubsystem, Loggable {
    private final Pigeon gyro;
    private final SwerveDriveOdometry odometry;
    private DrivetrainConstants constants;
    private SwerveDriveKinematics kinematics;
    private SwerveModule[] modules;

    public DrivetrainSS(DrivetrainConstants constants) {
        this.constants = constants;
        this.gyro = constants.CAN_MAP.GYRO;
        initSwerve();
        this.odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());
        gyro.calibrate();
        gyro.reset();
    }

    /**
     * Drives the system with the given speeds
     *
     * @param speeds the ChassisSpeeds object representing the desired speeds
     */
    public void
    speedDrive(ChassisSpeeds speeds) {
        SwerveModuleState[] states =
                kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(states, constants.MAX_SPEED_MPS);
        setDesiredStates(states);
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
        // Multiplying the wanted power by the max speed gives us the wanted speed
        x *= constants.MAX_SPEED_MPS;
        y *= constants.MAX_SPEED_MPS;
        rot *= constants.MAX_ROT_SPEED_RAD_S;
        // We intentionally switch x and y because Rotation2D uses x as forward and y as sideways
        speedDrive(new ChassisSpeeds(y, x, rot));
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
        // Multiplying the wanted power by the max speed gives us the wanted speed
        x *= constants.MAX_SPEED_MPS;
        y *= constants.MAX_SPEED_MPS;
        rot *= constants.MAX_ROT_SPEED_RAD_S;
        // We intentionally switch x and y because Rotation2D uses x as forward and y as sideways
        fieldSpeedDrive(new ChassisSpeeds(y, x, rot));
    }

    /**
     * Returns the desired states of the swerve modules as an array
     *
     * @return the modules' desired states
     */
    public SwerveModuleState[] getDesiredStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getDesiredState();
        }
        return states;
    }

    /**
     * Sets the swerve modules' desired states
     *
     * @param states the modules' desired states
     */
    public void setDesiredStates(SwerveModuleState[] states) {
        boolean hasError = false;
        for (SwerveModule module : modules) {
            if (module.getAngleError() > 40) {
                hasError = true;
                break;
            }
        }
        if (hasError) {
            for (int i = 0; i < modules.length; i++) {
                states[i].speedMetersPerSecond = 0;
            }
        }
        if (states.length != modules.length)
            return;
        for (int i = 0; i < states.length; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    /**
     * Returns the current states of the swerve modules as an array
     *
     * @return the modules' current states
     */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the angle of the system in degrees
     *
     * @return the angle of the system
     */
    @Log(name = "Angle")
    public double getAngle() {
        double angle = gyro.getAngle();
        angle = (360 - angle);
        while (angle < 0)
            angle += 360;
        while (angle >= 360)
            angle -= 360;
        return angle;
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
    @Log(name = "X")
    public double getX() {
        return getPose().getX();
    }

    /**
     * @return the y position on the field as calculated by the odometry
     */
    @Log(name = "Y")
    public double getY() {
        return getPose().getY();
    }

    /**
    * Resets the odometry to the specified pose.
    *
    * @param pose The pose to which to set the odometry.
    */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, gyro.getRotation2d());
    }


    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void SetSpeedMotorRampRates(double rampRate) {
        for (SwerveModule swerveModule : modules) {
            swerveModule.setSpeedMotorRampRate(rampRate);
        }
    }

    /*
     * Sets the ramp rate of the speed motors to the defualt ramp rate.
     */
    public void SetSpeedMotorRampRates() {
        for (SwerveModule swerveModule : modules) {
            swerveModule.setSpeedMotorRampRate();
        }
    }

    @Override
    public void periodic() {
        updateOdometry();
       for (SwerveModule module : modules)
           module.periodic();
        SmartDashboard.putNumber("angle", getAngle());
    }

    private void updateOdometry() {
        odometry.update(gyro.getRotation2d(), getDesiredStates());
    }

    private void initSwerve() {
        kinematics = new SwerveDriveKinematics(
                constants.FRONT_LEFT_LOCATION.getTranslation(),
                constants.FRONT_RIGHT_LOCATION.getTranslation(),
                constants.REAR_LEFT_LOCATION.getTranslation(),
                constants.REAR_RIGHT_LOCATION.getTranslation()
        );
        modules = new SwerveModule[]{
                constants.CAN_MAP.FRONT_LEFT,
                constants.CAN_MAP.FRONT_RIGHT,
                constants.CAN_MAP.REAR_LEFT,
                constants.CAN_MAP.REAR_RIGHT
        };
        sendData("Front Left", modules[0]);
        sendData("Front Right", modules[1]);
        sendData("Rear Left", modules[2]);
        sendData("Rear Right", modules[3]);
    }

    private void sendData(String name, SwerveModule module) {
        ShuffleboardLayout layout = Shuffleboard.getTab("Swerve").getLayout(configureLogName() + "/" + name, BuiltInLayouts.kList);
        layout.add("Module stats", module);
        layout.add("Angle PID Controller", module.getAnglePIDController());
        layout.add("Speed PID Controller", module.getSpeedPIDController());
    }

    @Override
    public String configureLogName() {
        return "Drivetrain";
    }
}
