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

        desiredState = getState();

        angleController.enableContinuousInput(0, 360);

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
        // so we take the remainder.
        return (angleMotor.getSelectedSensorPosition()
                / SwerveConstants.StaticSwerveConstants.ANGLE_TICKS_PER_REVOLUTION * 360 - constants.offset) % 360;
    }

    private void setRelative() {
        angleMotor.configSelectedFeedbackSensor(SwerveConstants.StaticSwerveConstants.RELATIVE_DEVICE);
    }

    private void setAbsolute() {
        angleMotor.configSelectedFeedbackSensor(SwerveConstants.StaticSwerveConstants.ABSOLUTE_DEVICE);
    }

    public boolean isTuning() {
        return speedController.isTuning();
    }

    public void setIsTuning(boolean isTuning) {
        speedController.setIsTuning(isTuning);
        angleController.setIsTuning(isTuning);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Angle", this::getAngle, x -> {});
        builder.addDoubleProperty("Desired Angle", this::getAngle,
                angle -> desiredState.angle = Rotation2d.fromDegrees(angle));
        builder.addDoubleProperty("Velocity", this::getSpeedMotorMPS, x -> {});
        builder.addDoubleProperty("Desired Velocity", this::getSpeedMotorMPS,
                speed -> desiredState.speedMetersPerSecond = speed);
        speedController.initSendable(builder, "Speed Controller");
        angleController.initSendable(builder, "Angle Controller");
    }
}
