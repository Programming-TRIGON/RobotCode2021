package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.SubsystemContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drivetrain.SupplierDriveCMD;
import frc.robot.vision.limelights.PitcherLimelight;

public class BackupAuto extends SequentialCommandGroup {
    private static final double X_POWER = 0.0;
    private static final double Y_POWER = -0.2;
    private static final double Z_POWER = 0.0;
    private static final double SHOOT_TIMEOUT = 7;
    private static final double DRIVE_TIMEOUT = 1;
    
    private final SubsystemContainer subsystems;
    private final RobotConstants constants;
    private final PitcherLimelight limelight;

    public BackupAuto(SubsystemContainer subsystems, RobotConstants constants, PitcherLimelight limelight) {
        this.subsystems = subsystems;
        this.constants = constants;
        this.limelight = limelight;

        addRequirements(subsystems.DRIVETRAIN_SS, subsystems.SHOOTER_SS, subsystems.PITCHER_SS,
                subsystems.LOADER_SS, subsystems.SPINNER_SS);

        ShootCMDGP shootCMDGP = new ShootCMDGP(subsystems, constants, limelight);
        addCommands(
            new InstantCommand(() -> subsystems.PITCHER_SS.setSolenoidState(true)),
            shootCMDGP.withTimeout(SHOOT_TIMEOUT),
            new SupplierDriveCMD(subsystems.DRIVETRAIN_SS, () -> X_POWER, () -> Y_POWER, () -> Z_POWER).withTimeout(DRIVE_TIMEOUT),
            new InstantCommand(() -> subsystems.PITCHER_SS.setSolenoidState(false))
        );
    }
} 
