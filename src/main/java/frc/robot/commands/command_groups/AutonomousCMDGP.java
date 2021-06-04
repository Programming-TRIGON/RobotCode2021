package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.SubsystemContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drivetrain.SupplierDriveCMD;
import frc.robot.vision.limelights.PitcherLimelight;

public class AutonomousCMDGP extends SequentialCommandGroup {
    private final SubsystemContainer subsystems;
    private final RobotConstants constants;
    private final PitcherLimelight limelight;

    public AutonomousCMDGP(SubsystemContainer subsystems, RobotConstants constants, PitcherLimelight limelight) {
        this.subsystems = subsystems;
        this.constants = constants;
        this.limelight = limelight;

        addRequirements(subsystems.DRIVETRAIN_SS, subsystems.SHOOTER_SS, subsystems.PITCHER_SS,
                subsystems.LOADER_SS, subsystems.SPINNER_SS);

        ShootCMDGP shootCMDGP = new ShootCMDGP(subsystems, constants, limelight);
        shootCMDGP.enableBallInterupted(true);

        addCommands(
                        shootCMDGP,
                        new SupplierDriveCMD(subsystems.DRIVETRAIN_SS, () -> 0.0, () -> -0.2, () -> 0.0),
                        new WaitCommand(2)
        );
    }
}