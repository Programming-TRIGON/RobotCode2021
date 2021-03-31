package frc.robot;

import frc.robot.subsystems.drivetrain.DrivetrainSS;
import frc.robot.subsystems.intake.IntakeSS;
import frc.robot.subsystems.intake_opener.IntakeOpenerSS;
import frc.robot.subsystems.led.LedSS;
import frc.robot.subsystems.loader.LoaderSS;
import frc.robot.subsystems.pitcher.PitcherSS;
import frc.robot.subsystems.shooter.ShooterSS;
import frc.robot.subsystems.spinner.SpinnerSS;

public abstract class SubsytemContainer {
    public LedSS LED_SS;
    public DrivetrainSS DRIVETRAIN_SS;
    public ShooterSS SHOOTER_SS;
    public PitcherSS PITCHER_SS;
    public LoaderSS LOADER_SS;
    public SpinnerSS SPINNER_SS;
    public IntakeSS INTAKE_SS;
    public IntakeOpenerSS INTAKE_OPENER_SS;
}
