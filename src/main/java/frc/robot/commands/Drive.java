package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.camera.Limelight;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyro.Odometry;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Drive extends CommandBase {
    // Swerb controller
    private final SwerveDrive swerveSubsystem;
    // Limelight
    private final Limelight limelight;
    // Joystick Limiters & Values
    private final SlewRateLimiter vxFLimiter;
    private final SlewRateLimiter vyFLimiter;
    private final SlewRateLimiter rpsFLimiter;
    // Gyroscope
    private AHRS navX2;

    /**
     * Creates a new SwerveDriveCommand.
     * This is the main command class for the robot during Teleop.
     * It handles all the swerve drive and limelight functionality,
     * as well as the auto-leveling with the press of a button
     *
     * @param subsystem The subsystem used by this command.
     */

    private class FilteredValues{
        public double vx;
        public double vy;
        public double rps;
    }

    public Drive(SwerveDrive subsystem, Limelight limelight, AHRS navX2) {
        this.limelight = limelight;
        this.swerveSubsystem = subsystem;
        this.navX2 = navX2;
        addRequirements(subsystem);
        addRequirements(limelight);

        vxFLimiter = new SlewRateLimiter(4);
        vyFLimiter = new SlewRateLimiter(4);
        rpsFLimiter = new SlewRateLimiter(4);
        
        
        SmartDashboard.putNumber("Left Stick Angle (Radians)", 0);
        SmartDashboard.putNumber("Left Stick Magnitude", 0);
        SmartDashboard.putNumber("Right Stick Rotation", 0);

    
        SmartDashboard.putBoolean("Xbox Controller", true);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putString("Current Mode", "Drive");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // I do not know why this works. I wish it didn't
        // However, we added this random smart dashboard update now
        // the command doesn't automatically cancel itself
        // SmartDashboard.putString("Current Mode", "AAAAAAAAAAAAAAAA");

        // Get joystick values
        FilteredValues filteredDriveSpeeds = getFilteredValues();
        // Drive via stick values
        swerveSubsystem.drive(filteredDriveSpeeds.vx, -filteredDriveSpeeds.vy, filteredDriveSpeeds.rps,true);
        swerveSubsystem.debugPutEncoderValues();
        SmartDashboard.putNumber("navx X", navX2.getYaw());
        SmartDashboard.putNumber("navx Y", navX2.getPitch());
    }

    public FilteredValues getFilteredValues(){
        // Get Joystick Values vx and vy are in meters per second
        FilteredValues fv = new FilteredValues();
        if (SmartDashboard.getBoolean("Xbox Controller", true)) {
            fv.vx = Constants.driverController.getLeftX();
            fv.vy = Constants.driverController.getLeftY();
            fv.rps = Constants.driverController.getRightX();
        } else {
            fv.vx = Constants.driverJoystick.getX();
            fv.vy = Constants.driverJoystick.getY();
            fv.rps = Constants.driverJoystick.getThrottle(); 
        }

        // Controller stick drift deadband, limiters, and velocity multipliers
        if (Math.sqrt(fv.vx * fv.vx + fv.vy * fv.vy) < 0.2) {
            fv.vx = 0;
            fv.vy = 0;
        }

        fv.vx = vxFLimiter.calculate(fv.vx) * Constants.maxDriveSpeed;
        fv.vy = vyFLimiter.calculate(fv.vy) * Constants.maxDriveSpeed;
        //Comment in to disable rotation
        //fv.rps = 0;
        fv.rps = rpsFLimiter.calculate(fv.rps) * Constants.chassisRotationsPerSecondMultiplier;
        
        return fv;
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // swerveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
