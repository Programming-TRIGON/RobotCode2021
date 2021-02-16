package frc.robot.components;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.utilities.SwerveConstants;
import frc.robot.utilities.TrigonPIDController;

public class SwerveModule implements Sendable {
    private final TrigonTalonFX speedMotor;
    private final TrigonTalonFX angleMotor;
    private final TrigonPIDController speedController;
    private final TrigonPIDController angleController;
    private SwerveModuleState desiredState;
    private SwerveConstants constants;
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
        speedController = new TrigonPIDController(constants.speedCoefs);

        angleMotor = constants.angleMotor;
        angleController = new TrigonPIDController(constants.angleCoefs);

        setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(getAngle())));

        angleController.enableContinuousInput(0, 360);

        setAbsolute();

        isTuning = false;
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
        speedMotor.set(speedController.calculate(getSpeedMotorMPS()));
        angleMotor.set(angleController.calculate(getAngle()));
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
        updateSetpoint();
    }

    /**
     * Sets the desired angle of the module. This won't affect the motors' power
     * until the next periodic run
     *
     * @param desiredAngle the desired angle for the module in degrees.
     */
    public void setDesiredAngle(double desiredAngle) {
        SwerveModuleState newDesiredState = new SwerveModuleState(desiredState.speedMetersPerSecond,
                Rotation2d.fromDegrees(getAngle()));
        desiredState = SwerveModuleState.optimize(newDesiredState, Rotation2d.fromDegrees(getAngle()));
        updateSetpoint();
    }

    /**
     * Sets the desired speed of the module. This won't affect the motors' power
     * until the next periodic run
     *
     * @param desiredSpeed the desired speed for the module in meters per second.
     */
    public void setDesiredSpeed(double desiredSpeed) {
        SwerveModuleState newDesiredState = new SwerveModuleState(desiredSpeed, desiredState.angle);
        desiredState = SwerveModuleState.optimize(newDesiredState, Rotation2d.fromDegrees(getAngle()));
        updateSetpoint();
    }

    /**
     * @return the current state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeedMotorMPS(), Rotation2d.fromDegrees(getAngle()));
    }

    /**
     * Update the PID controllers' set point according to the desired state
     */
    public void updateSetpoint() {
        speedController.setSetpoint(desiredState.speedMetersPerSecond);
        angleController.setSetpoint(desiredState.angle.getDegrees());
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
                * constants.diameter * Math.PI / 10;
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
        return 360-(angleMotor.getSelectedSensorPosition()
                / (SwerveConstants.StaticSwerveConstants.ANGLE_TICKS_PER_REVOLUTION - 1) * 360 - constants.offset);
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
        speedController.setIsTuning(isTuning);
        angleController.setIsTuning(isTuning);
        this.isTuning = isTuning;
    }

    public TrigonPIDController getSpeedPIDController() {
        return speedController;
    }

    public TrigonPIDController getAnglePIDController() {
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
    }
}
