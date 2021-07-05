package frc.robot.components;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.Logger;
import frc.robot.utilities.SwerveConstants;
import frc.robot.utilities.SwerveConstants.StaticSwerveConstants;
import frc.robot.utilities.TrigonPIDController;
import frc.robot.utilities.TrigonProfiledPIDController;

public class SwerveModule implements Sendable {
    private final SwerveConstants constants;
    private final TrigonTalonFX angleMotor;
    private final TrigonTalonFX speedMotor;
    private final TrigonProfiledPIDController angleController;
    private final TrigonPIDController aController;
    private final TrigonPIDController speedController;
    private final SimpleMotorFeedforward angleFeedforward;
    private final SimpleMotorFeedforward speedFeedforward;
    private SwerveModuleState desiredState;
    private boolean isTuning;
    private boolean move;
    private Logger logger;
    private boolean startedLogging;

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
        speedFeedforward = new SimpleMotorFeedforward(constants.speedSvaCoefs.getKS(), constants.speedSvaCoefs.getKV(),
                constants.speedSvaCoefs.getKA());

        angleMotor = constants.angleMotor;
        angleController = new TrigonProfiledPIDController(constants.anglePidfCoefs);
        aController = new TrigonPIDController(constants.anglePidfCoefs);
        angleFeedforward = new SimpleMotorFeedforward(constants.angleSvaCoefs.getKS(), constants.angleSvaCoefs.getKV(),
                constants.angleSvaCoefs.getKA());

        setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(getAngle())));

        angleController.enableContinuousInput(-90, 90);
        aController.enableContinuousInput(-90, 90);
        setAbsolute();
        speedMotor.setSelectedSensorPosition(0);
        move = false;
        SmartDashboard.putBoolean("swerve/move", false);
        SmartDashboard.putNumber("swerve/power", 0);
    //    setDesiredAngle(0);

        logger = new Logger("Swerve test angle motor: " + angleMotor.getDeviceID(), "Power", "Velocity");
        startedLogging = false;
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
        if (getDesiredVelocity() != 0)
            speedMotor.setVoltage(speedController.calculate(getSpeedMotorMPS(), getDesiredVelocity())
                    + speedFeedforward.calculate(getDesiredVelocity()));
        else
            speedMotor.set(0);
        // double pid = angleController.calculate(getAngle(), desiredState.angle.getDegrees());
        // angleMotor.setVoltage(pid + angleFeedforward.calculate(angleController.getSetpoint().velocity));
        aController.setSetpoint(desiredState.angle.getDegrees());
        angleMotor.setVoltage(aController.calculate(getAngle()));

        // The code below is used for testing, it is very disgusting but for now it must do
        // !!!!!!!DO NOT UNCOMMENT IF NOT TESTING!!!!!WHEN FINSHED TESTING RECOMMENT!!!!!

    //     // double pid = angleController.calculate(getAngle(), SmartDashboard.getNumber("swerve/power", 0));
    //     System.out.println("PID: " + pid);
    //    angleMotor.setVoltage(angleFeedforward.calculate(angleController.getSetpoint().velocity) + pid);
    //     move = SmartDashboard.getBoolean("swerve/move", false);
    //     if (move) {
    //        speedController.setSetpoint(SmartDashboard.getNumber("swerve/power", 0));
    //        double speedPID = speedController.calculateWithKF(getSpeedMotorMPS());
    //    speedMotor.setVoltage(speedPID);
    //         // angleMotor.setVoltage(angleFeedforward.calculate(SmartDashboard.getNumber("swerve/power", 0)));
    //        startedLogging = true;
    //        logger.log(SmartDashboard.getNumber("swerve/power", 0), getSpeedMotorMPS());
    //    } else if (startedLogging) {
    //        startedLogging = false;
    //        logger.close();
    //    }
        if (angleMotor.getDeviceID() == 3) {
            SmartDashboard.putNumber("Front Left angleController.getSetpoint().velocity",
                    angleController.getSetpoint().velocity);
            SmartDashboard.putNumber("Front Left angleFeedforward.calculate(angleController.getSetpoint().velocity)",
                    angleFeedforward.calculate(angleController.getSetpoint().velocity));
            // SmartDashboard.putNumber("Front Left pid", pid);
            SmartDashboard.putNumber("Front Left error", getAngleError());
            SmartDashboard.putNumber("Front Left Angle vel mistake",
                    angleController.getSetpoint().velocity - getAngleMotorAPS());
            SmartDashboard.putNumber("Front Left Speed vel mistake", getDesiredVelocity() - getSpeedMotorMPS());
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
        // return Math.abs(getAngle() - angleController.getGoal().position);
        return aController.getPositionError();
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
        aController.setIsTuning(isTuning);
        this.isTuning = isTuning;
    }

    public TrigonPIDController getAnglePIDController() {
        return aController;
    }

    public TrigonPIDController getSpeedPIDController() {
        return speedController;
    }

    public void setSpeedMotorRampRate(double rampRate) {
        speedMotor.configOpenloopRamp(rampRate);
        speedMotor.configClosedloopRamp(rampRate);
    }

    public void setMotorsNeutralMode(NeutralMode mode) {
        speedMotor.setNeutralMode(mode);
        angleMotor.setNeutralMode(mode);
    }

    public void toggleMotorsNeutralMode() {
        setMotorsNeutralMode(angleMotor.getNeutralMode() == NeutralMode.Coast ? NeutralMode.Brake : NeutralMode.Coast);
        System.out.println("Drivetrain/onBrake:\t" + (angleMotor.getNeutralMode() == NeutralMode.Brake));
    }

    /*
     * Sets the ramp rate of the speed motor to the default ramp rate.
     */
    public void setSpeedMotorRampRate() {
        speedMotor.configOpenloopRamp(StaticSwerveConstants.SPEED_DEFAULT_CONFIG.getRampRate());
        speedMotor.configClosedloopRamp(StaticSwerveConstants.SPEED_DEFAULT_CONFIG.getRampRate());
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
        // builder.addDoubleProperty("PID Angle Velocity", () -> angleController.getSetpoint().velocity, null);

        builder.addBooleanProperty("isTuning", this::isTuning, this::setIsTuning);
        builder.addDoubleProperty("Speed Motor Position", () -> speedMotor.getSelectedSensorPosition() / SwerveConstants.StaticSwerveConstants.SPEED_MOTOR_TICKS_PER_REVOLUTION
                * constants.diameter * Math.PI / SwerveConstants.StaticSwerveConstants.SPEED_GEAR_RATION, null);
        builder.addDoubleProperty("Angle PID error", this::getAngleError, null);
        builder.addDoubleProperty("Speed PID error", () -> getSpeedMotorMPS() - getDesiredVelocity(), null);
        builder.addDoubleProperty("Angle motor power", this.angleMotor::get, null);
    }
}
