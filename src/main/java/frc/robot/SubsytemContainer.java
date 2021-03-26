package frc.robot;

import frc.robot.subsystems.drivetrain.DrivetrainSS;
import frc.robot.subsystems.intake.IntakeSS;
import frc.robot.subsystems.intakeOpener.IntakeOpenerSS;
import frc.robot.subsystems.led.LedSS;
import frc.robot.subsystems.loader.LoaderSS;
import frc.robot.subsystems.pitcher.PitcherSS;
import frc.robot.subsystems.shooter.ShooterSS;
import frc.robot.subsystems.spinner.SpinnerSS;

public abstract class SubsytemContainer {
    public LedSS LEDSS;
    public DrivetrainSS DRIVETRAINSS;
    public ShooterSS SHOOTERSS;
    public PitcherSS PITCHERSS;
    public LoaderSS LOADERSS;
    public SpinnerSS SPINNERSS;
    public IntakeSS INTAKESS;
    public IntakeOpenerSS INTAKEOPENERSS;
}
