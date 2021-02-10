package frc.robot.components;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.constants.RobotConstants;
import frc.robot.utilities.TrigonPIDController;

public class SwerveModule {
    private final TrigonTalonFX
            speedMotor,
            angleMotor;
    private final TrigonPIDController
            speedController,
            angleController;
    private SwerveModuleState state;
    private double wheelDiameter, angleOffset;

    /**
     * Constructs a swerve module that's is made of a speed motor and an angle motor.
     *
     * @param speedID       the motor ID for the speed motor
     * @param angleID       the motor ID for the angle motor
     * @param wheelDiameter the diameter of the wheel of the module
     * @param angleOffset   the angle that the encoder will give when the wheel itself is on 0
     */
    public SwerveModule(int speedID, int angleID, double wheelDiameter, double angleOffset) {
        this.wheelDiameter = wheelDiameter;
        this.angleOffset = angleOffset;
        speedMotor = new TrigonTalonFX(speedID, RobotConstants.swerveConstants.SWERVE_MODULE_SPEED_MOTOR_CONFIG);
        speedController = new TrigonPIDController(RobotConstants.swerveConstants.SWERVE_MODULE_SPEED_PID_COEFS);

        angleMotor = new TrigonTalonFX(angleID, RobotConstants.swerveConstants.SWERVE_MODULE_ANGLE_MOTOR_CONFIG);
        angleController = new TrigonPIDController(RobotConstants.swerveConstants.SWERVE_MODULE_ANGLE_PID_COEFS);
        angleMotor.configSelectedFeedbackSensor(RobotConstants.swerveConstants.SWERVE_MODULE_ANGLE_MOTOR_FEEDBACK_DEVICE);

    }

    /**
     * Returns the ticks of the speed motor
     *
     * @return the speed motor's ticks
     */
    public int getSpeedMotorTicks() {
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
     * @return the wanted state of the module
     */
    public SwerveModuleState getState() {
        return state;
    }

    /**
     * Sets the wanted state of the module.
     * This won't affect the motors' power until the next periodic run
     *
     * @param state the wanted state for the module.
     */
    public void setState(SwerveModuleState state) {
        this.state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAngle()));
        updateSetpoint();
    }

    /**
     * Update the PID controllers' set point according to the wanted state
     */
    public void updateSetpoint() {
        speedController.setSetpoint(state.speedMetersPerSecond);
        angleController.setSetpoint(state.angle.getDegrees());
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
        return getSpeedMotorVelocity() / 10 / RobotConstants.swerveConstants.TALON_FX_TICKS_PER_REVOLUTION * wheelDiameter;
    }

    /**
     * @return the angle of the module in degrees
     */
    public double getAngle() {
        return angleMotor.getSelectedSensorPosition() / RobotConstants.swerveConstants.TALON_FX_TICKS_PER_REVOLUTION * 360 - angleOffset;
    }
}
