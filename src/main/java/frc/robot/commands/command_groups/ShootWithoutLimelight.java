// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.command_groups;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SubsystemContainer;
import frc.robot.commands.TurnToTargetCMD;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.loader.LoaderCMD;
import frc.robot.subsystems.shooter.ShooterCMD;
import frc.robot.subsystems.spinner.SpinnerCMD;

public class ShootWithoutLimelight extends ParallelCommandGroup {
    public ShootWithoutLimelight(SubsystemContainer subsystems, RobotConstants constants, DoubleSupplier sup) {

        ShooterCMD shootCMD = new ShooterCMD(subsystems.SHOOTER_SS, subsystems.LED_SS, constants.shooterConstants, sup);
        addCommands(shootCMD,
                new SequentialCommandGroup(
                        new WaitUntilCommand(shootCMD::isAtSetpoint),
                        new ParallelCommandGroup(
                                new LoaderCMD(subsystems.LOADER_SS, constants.loaderConstants,
                                        () -> SmartDashboard.getNumber("Loader Vel", constants.loaderConstants.DEFAULT_SHOOTING_VELOCITY)),
                                new SpinnerCMD(subsystems.SPINNER_SS, constants.spinnerConstants))));
    }
}
