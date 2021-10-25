// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LiftPWMCMD extends CommandBase {
  private final LiftSS liftSS;
  private DoubleSupplier power;
  private int powerCount;

  /** Creates a new LiftPWMCMD. */
  public LiftPWMCMD(LiftSS liftSS, DoubleSupplier power) {
    this.liftSS = liftSS;
    this.power = power;
    addRequirements(liftSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    powerCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (power.getAsDouble() > 0) {
      if (powerCount < 2) {
        liftSS.move(power.getAsDouble());
        powerCount++;
      } else {
        liftSS.stopMoving();
        if (powerCount < 3)
          powerCount++;
        else
          powerCount = 0;
      }
    }
    else
      liftSS.move(power.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    liftSS.stopMoving();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
