// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SubsystemContainer;
import frc.robot.commands.TurnToTargetCMD;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.loader.LoaderCMD;
import frc.robot.subsystems.shooter.ShooterCMD;
import frc.robot.subsystems.spinner.SpinnerCMD;
import frc.robot.vision.Target;
import frc.robot.vision.limelights.PitcherLimelight;

public class ShootCMDGP extends ParallelCommandGroup {
        private ShooterCMD shootCMD;
    public ShootCMDGP(SubsystemContainer subsystems, RobotConstants constants, PitcherLimelight limelight) {
        addRequirements(subsystems.DRIVETRAIN_SS,subsystems.SHOOTER_SS, subsystems.PITCHER_SS, subsystems.LOADER_SS, subsystems.SPINNER_SS);

        shootCMD = new ShooterCMD(subsystems.SHOOTER_SS, subsystems.LED_SS, constants.shooterConstants, ()->0);
        addCommands(new ParallelDeadlineGroup(
                        shootCMD,
                        new SequentialCommandGroup(
                                new TurnToTargetCMD(subsystems.DRIVETRAIN_SS, limelight, constants.visionConstants, Target.PowerPort),
                                new WaitUntilCommand(shootCMD::isAtSetpoint),
                                new ParallelCommandGroup(
                                        new LoaderCMD(subsystems.LOADER_SS, constants.loaderConstants, constants.loaderConstants.DEFAULT_SHOOTING_VELOCITY),
                                        new SpinnerCMD(subsystems.SPINNER_SS, constants.spinnerConstants)
                                )
                        )
                )
        );
    }

    public void enableBallInterupted(boolean enableBallInterupted) {
        shootCMD.enableBallInterupted(enableBallInterupted);
    }
}
