package frc.robot.components;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.FeedforwardConstants;
import frc.robot.utilities.SwerveConstants;
import frc.robot.utilities.TrigonPIDController;
import frc.robot.utilities.TrigonProfiledPIDController;

public class SwerveModule implements Sendable {
    private final SwerveConstants constants;
    private final TrigonTalonFX angleMotor;
    private final TrigonTalonFX speedMotor;
    private final TrigonProfiledPIDController angleController;
    private final TrigonPIDController speedController;
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
        speedController = new TrigonPIDController(constants.speedPidfCoefs);

        angleMotor = constants.angleMotor;
        angleController = new TrigonProfiledPIDController(constants.anglePidfCoefs);

        setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(getAngle())));

        angleController.enableContinuousInput(-90, 90);

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
    public void periodic(Command currentCommand) {
        speedMotor.setVoltage(speedController.calculate(getSpeedMotorMPS(), getDesiredVelocity())
                + constants.speedFeedForwardConstants.mCoef * getDesiredVelocity()
                + constants.speedFeedForwardConstants.bCoef);
        if (!currentCommand.getName().equals("CalibratesSpeedKf")) {
            double pid = angleController.calculate(getAngle(), desiredState.angle.getDegrees());
            angleMotor.setVoltage(pid + constants.angleFeedForwardConstants.mCoef * angleController.getSetpoint().velocity
                    + constants.angleFeedForwardConstants.bCoef);
            if (angleMotor.getDeviceID() == 3) {
                SmartDashboard.putNumber("Front Left angleController.getSetpoint().velocity",
                        angleController.getSetpoint().velocity);
                SmartDashboard.putNumber("Front Left pid", pid);
                SmartDashboard.putNumber("Front Left error", getAngleError());
            }
        }
        if (desiredState.angle.getDegrees() == 0) {
            desiredState.angle = Rotation2d.fromDegrees(getAngle());
        }
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

    public double getAngleError() {
        return Math.abs(getAngle() - angleController.getGoal().position);
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

    public double getAngleMotorVelocity() {
        return angleMotor.getSelectedSensorVelocity();
    }

    public double getAngleMotorAPS() {
        return getAngleMotorVelocity() * 10 / 2048 / 12.8 * 360;
    }

    /**
     * @return the angle of the module in degrees
     */
    public double getAngle() {
        // The position of the sensor gives us the specific tick we are on from 0 - tpr.
        // Dividing by tpr gives us a number between 0 and 1. multiplying by 360 gives
        // us the degrees.
        // If the result is 360 we want it to be 0 and if it's 400 we want it to be 40
        int tpr = SwerveConstants.StaticSwerveConstants.ANGLE_TICKS_PER_REVOLUTION;
        double pos = Math.abs(angleMotor.getSelectedSensorPosition() - tpr) - (constants.offset / 360 * tpr);
        if (pos < 0)
            pos += tpr;
        return pos / tpr * 360;
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

    /**
     * Sets the voltage output of the the angle motor. Compensates for the current
     * bus voltage to ensure that the desired voltage is output even if the battery
     * voltage is below 12V - highly useful when the voltage outputs are
     * "meaningful" (e.g. they come from a feedforward calculation).
     *
     * <p>
     * NOTE: This function *must* be called regularly in order for voltage
     * compensation to work properly - unlike the ordinary set function, it is not
     * "set it and forget it."
     *
     * @param outputVolts The voltage to output.
     */
    public void setAngleMotorVoltage(double outputVolts) {
        angleMotor.setVoltage(outputVolts);
    }

    /**
     * Sets the voltage output of the the speed motor. Compensates for the current
     * bus voltage to ensure that the desired voltage is output even if the battery
     * voltage is below 12V - highly useful when the voltage outputs are
     * "meaningful" (e.g. they come from a feedforward calculation).
     *
     * <p>
     * NOTE: This function *must* be called regularly in order for voltage
     * compensation to work properly - unlike the ordinary set function, it is not
     * "set it and forget it."
     *
     * @param outputVolts The voltage to output.
     */
    public void setSpeedMotorVoltage(double outputVolts) {
        speedMotor.setVoltage(outputVolts);
    }

    public FeedforwardConstants getAngleFeedforwardConstants() {
        return constants.angleFeedForwardConstants;
    }

    public FeedforwardConstants getSpeedFeedforwardConstants() {
        return constants.speedFeedForwardConstants;
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
