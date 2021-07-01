package frc.robot;

import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.climber.LiftSS;
import frc.robot.subsystems.climber.WinchSS;
import frc.robot.subsystems.drivetrain.DrivetrainSS;
import frc.robot.subsystems.intake.IntakeSS;
import frc.robot.subsystems.intake_opener.IntakeOpenerSS;
import frc.robot.subsystems.loader.LoaderSS;
import frc.robot.subsystems.pitcher.PitcherSS;
import frc.robot.subsystems.shooter.ShooterSS;
import frc.robot.subsystems.spinner.SpinnerSS;

public class SubsystemContainerA extends SubsystemContainer {
    public SubsystemContainerA(RobotConstants robotConstants) {
        // TODO: Set to subsystem when LEDs are added to the robot
        LED_SS = null;
        DRIVETRAIN_SS = new DrivetrainSS(robotConstants.drivetrainConstants);
        SHOOTER_SS = new ShooterSS(robotConstants.shooterConstants);
        PITCHER_SS = new PitcherSS(robotConstants.pitcherConstants);
        LOADER_SS = new LoaderSS(robotConstants.loaderConstants);
        SPINNER_SS = new SpinnerSS(robotConstants.spinnerConstants);
        INTAKE_SS = new IntakeSS(robotConstants.intakeConstants);
        INTAKE_OPENER_SS = new IntakeOpenerSS(robotConstants.intakeOpenerConstants);
        LIFT_SS = new LiftSS(robotConstants.climberConstants);
        WINCH_SS = new WinchSS(robotConstants.climberConstants);
    }
}