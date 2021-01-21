// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants;

public class IntakeSS extends SubsystemBase {
 private TrigonTalonSRX motor;


  public IntakeSS(RobotConstants.IntakeConstants intakeConstants) {
motor=new TrigonTalonSRX(intakeConstants.INTAKE_CAN_MAP.MOTOR_ID,intakeConstants.MOTOR_CONFIG)
  }

 public void setMotor(double power){
motor.set(power);

 }
}
