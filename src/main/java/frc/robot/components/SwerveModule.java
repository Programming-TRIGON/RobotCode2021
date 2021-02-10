package frc.robot.components;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.constants.RobotConstants;
import frc.robot.utilities.SwerveConstants;
import frc.robot.utilities.TrigonPIDController;

public class SwerveModule {
    private final TrigonTalonFX
            speedMotor,
            angleMotor;
    private final TrigonPIDController
            speedController,
            angleController;
    private SwerveModuleState desiredState;
    private SwerveConstants constants;

    /**
     * Constructs a swerve module that's is made of a speed motor and an angle motor.
     *
     * @param constants the constants for this module
     */
    public SwerveModule(SwerveConstants constants) {
        this.constants = constants;
        speedMotor = new TrigonTalonFX(constants.speedID, RobotConstants.StaticSwerveConstants.SWERVE_MODULE_SPEED_MOTOR_CONFIG);
        speedController = new TrigonPIDController(constants.speedCoefs);

        angleMotor = new TrigonTalonFX(constants.angleID, RobotConstants.StaticSwerveConstants.SWERVE_MODULE_ANGLE_MOTOR_CONFIG);
        angleController = new TrigonPIDController(constants.angleCoefs);
        angleMotor.configSelectedFeedbackSensor(RobotConstants.StaticSwerveConstants.SWERVE_MODULE_ANGLE_MOTOR_FEEDBACK_DEVICE);
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
     * @return the desired state of the module
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Sets the desired state of the module.
     * This won't affect the motors' power until the next periodic run
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
        //Motor velocity in ticks/s divided by the ticks per revolution gives us the revolutions/s.
        // Multiplying by the circumference gives us the m/s
        return getSpeedMotorVelocity() / RobotConstants.StaticSwerveConstants.TALON_FX_TICKS_PER_REVOLUTION * constants.diameter * Math.PI;
    }

    /**
     * @return the angle of the module in degrees
     */
    public double getAngle() {
        // The position of the sensor gives us the specific tick we are on from 0 - tpr.
        // Dividing by tpr gives us a number between 0 and 1. multiplying by 360 gives us the degrees.
        return angleMotor.getSelectedSensorPosition() / RobotConstants.StaticSwerveConstants.TALON_FX_TICKS_PER_REVOLUTION * 360 - constants.offset;
    }
}
