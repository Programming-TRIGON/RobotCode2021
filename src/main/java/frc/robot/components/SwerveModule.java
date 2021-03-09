package frc.robot.components;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.utilities.SwerveConstants;
import frc.robot.utilities.TrigonProfiledPIDController;

public class SwerveModule implements Sendable {
    private final SwerveConstants constants;
    private final TrigonTalonFX angleMotor;
    private final TrigonTalonFX speedMotor;
    private final TrigonProfiledPIDController angleController;
    private final TrigonProfiledPIDController speedController;
    private final SimpleMotorFeedforward angleFeedforward;
    private final SimpleMotorFeedforward speedFeedforward;
    private SwerveModuleState desiredState;
    private boolean isTuning;

    /**
     * Constructs a swerve module that's is made of a speed motor and an angle
     * motor.
     *
     * @param constants the constants for this module
     */
    public SwerveModule(SwerveConstants constants) {
        this.constants = constants;
        speedMotor = constants.speedMotor;
        speedController = new TrigonProfiledPIDController(constants.speedCoefs);
        speedFeedforward = new SimpleMotorFeedforward(constants.speedCoefs.getKS(), constants.speedCoefs.getKV(), constants.speedCoefs.getKA());

        angleMotor = constants.angleMotor;
        angleController = new TrigonProfiledPIDController(constants.angleCoefs);
        angleFeedforward = new SimpleMotorFeedforward(constants.speedCoefs.getKS(), constants.speedCoefs.getKV(), constants.speedCoefs.getKA());

        setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(getAngle())));

        angleController.enableContinuousInput(-180, 180);

        setAbsolute();
    }

    /**
     * Returns the ticks of the speed motor
     *
     * @return the speed motor's ticks
     */
    public double getSpeedMotorTicks() {
        return speedMotor.getSelectedSensorPosition();
    }

    /**
     * Updates the PID controllers and sets the motors power
     */
    public void periodic() {
        speedMotor.set(speedController.calculate(getSpeedMotorVelocity()) + speedFeedforward.calculate(getDesiredVelocity()));
        angleMotor.set(angleController.calculate(getAngle(), desiredState.angle.getDegrees()));
    }

    /**
     * @return the desired state of the module
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Sets the desired state of the module. This won't affect the motors' power
     * until the next periodic run
     *
     * @param desiredState the desired state for the module.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getAngle()));
    }

    /**
     * Sets the desired angle of the module. This won't affect the motors' power
     * until the next periodic run
     *
     * @param desiredAngle the desired angle for the module in degrees.
     */
    public void setDesiredAngle(double desiredAngle) {
        desiredState.angle = Rotation2d.fromDegrees(desiredAngle);
        setDesiredState(desiredState);
    }

    /**
     * Sets the desired speed of the module. This won't affect the motors' power
     * until the next periodic run
     *
     * @param desiredSpeed the desired speed for the module in meters per second.
     */
    public void setDesiredSpeed(double desiredSpeed) {
        desiredState.speedMetersPerSecond = desiredSpeed;
        setDesiredState(desiredState);
    }

    /**
     * @return the current state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeedMotorMPS(), Rotation2d.fromDegrees(getAngle()));
    }

    /**
     * @return the speed of the module in ticks / 100ms
     */
    public double getSpeedMotorVelocity() {
        return speedMotor.getSelectedSensorVelocity();
    }

    /**
     * @return the speed of the module in m/s
     */
    public double getSpeedMotorMPS() {
        // Motor velocity in ticks/s divided by the ticks per revolution gives us the
        // revolutions/s.
        // Multiplying by the circumference gives us the m/s
        return getSpeedMotorVelocity() / SwerveConstants.StaticSwerveConstants.SPEED_MOTOR_TICKS_PER_REVOLUTION
                * constants.diameter * Math.PI * 10 / SwerveConstants.StaticSwerveConstants.SPEED_GEAR_RATION;
    }

    public double getDesiredVelocity() {
        return desiredState.speedMetersPerSecond;
    }

    /**
     * @return the angle of the module in degrees
     */
    public double getAngle() {
        // The position of the sensor gives us the specific tick we are on from 0 - tpr.
        // Dividing by tpr gives us a number between 0 and 1. multiplying by 360 gives
        // us the degrees.
        // If the result is 360 we want it to be 0 and if it's 400 we want it to be 40
        double pos = Math.abs(angleMotor.getSelectedSensorPosition() - 4095) - (constants.offset / 360 * (SwerveConstants.StaticSwerveConstants.ANGLE_TICKS_PER_REVOLUTION - 1));
        if (pos < 0)
            pos += 4095;
        return pos / (SwerveConstants.StaticSwerveConstants.ANGLE_TICKS_PER_REVOLUTION - 1) * 360;
    }

    private void setRelative() {
        angleMotor.configSelectedFeedbackSensor(SwerveConstants.StaticSwerveConstants.RELATIVE_DEVICE);
    }

    private void setAbsolute() {
        angleMotor.configSelectedFeedbackSensor(SwerveConstants.StaticSwerveConstants.ABSOLUTE_DEVICE);
        angleMotor.configFeedbackNotContinuous(true, 0);
    }

    public boolean isTuning() {
        return isTuning;
    }

    public void setIsTuning(boolean isTuning) {
        angleController.setIsTuning(isTuning);
        this.isTuning = isTuning;
    }

    public TrigonProfiledPIDController getAnglePIDController() {
        return angleController;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty("Current Angle", this::getAngle, null);
        builder.addDoubleProperty("Desired Angle", () -> getDesiredState().angle.getDegrees(),
                angle -> setDesiredAngle(isTuning ? angle : getDesiredState().angle.getDegrees()));
        builder.addDoubleProperty("Current Velocity", this::getSpeedMotorMPS, null);
        builder.addDoubleProperty("Desired Velocity", () -> getDesiredState().speedMetersPerSecond,
                speed -> setDesiredSpeed(isTuning ? speed : desiredState.speedMetersPerSecond));
        builder.addBooleanProperty("isTuning", this::isTuning, this::setIsTuning);
        builder.addDoubleProperty("Angle PID error", this.angleController::getPositionError, null);
        builder.addDoubleProperty("Angle motor power", this.angleMotor::get, null);
    }
}
