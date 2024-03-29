package frc.robot.commands.auton.paths;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoLevel;
import frc.robot.commands.auton.basic.DriveDistanceForward;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class Engage extends SequentialCommandGroup{

    private final SwerveDrive swerveDrive;
    private final AHRS navX2;
    public Engage(SwerveDrive swerveDrive, AHRS navX2) {
        this.swerveDrive = swerveDrive;
        this.navX2 = navX2;
        addRequirements(swerveDrive);

        addCommands(
            new DriveDistanceForward(swerveDrive, navX2, 3, 1),
            new AutoLevel(navX2, swerveDrive)
        );
    }


}