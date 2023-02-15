package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// Subsystem controlling all four individual swerve modules
public class SwerveDrive extends SubsystemBase {
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;

    private SwerveDriveKinematics kinematics;

    /** Creates a new ExampleSubsystem. */
    public SwerveDrive(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft,
            SwerveModule backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        kinematics = new SwerveDriveKinematics(
                Constants.frontLeftOffsetMeters,
                Constants.frontRightOffsetMeters,
                Constants.backLeftOffsetMeters,
                Constants.backRightOffsetMeters);
        SmartDashboard.putBoolean("Front Right Motor", true);
        SmartDashboard.putBoolean("Front Left Motor", true);
        SmartDashboard.putBoolean("Back Right Motor", true);
        SmartDashboard.putBoolean("Back Left Motor", true);
        SmartDashboard.putBoolean("Disable All Motors", false);
    }

    public void drive(double vx, double vy, double rotationRadiansPerSecond) {
        // Joystick values to a speed vector
        // Convert speed vector and rotation to module speeds
        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, rotationRadiansPerSecond);
        // Unpack module speeds
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveModuleState frontLeftState = states[0];
        SwerveModuleState frontRightState = states[1];
        SwerveModuleState backLeftState = states[2];
        SwerveModuleState backRightState = states[3];

        // stops modules from resetting their position if the joystick is not being used
        if (Math.sqrt(vx * vx + vy * vy) < 0.05 && Math.abs(rotationRadiansPerSecond) < 0.05) {
            stop();
            return;
        }

        // Override to enable all motors at once on SmartDashboard, on by default
        boolean disableAllMotors = SmartDashboard.getBoolean("Disable All Motors", false);
        /* Set swerve module speeds and rotations, also put them to smartdashboard */
        if (SmartDashboard.getBoolean("Front Right Motor", true) && !disableAllMotors) {
            // Actual Driving
            frontRight.drive(frontRightState);
        }
        if (SmartDashboard.getBoolean("Front Left Motor", true) && !disableAllMotors) {
            frontLeft.drive(frontLeftState);
        }
        if (SmartDashboard.getBoolean("Back Right Motor", true) && !disableAllMotors) {
            backRight.drive(backRightState);
        }
        if (SmartDashboard.getBoolean("Back Left Motor", true) && !disableAllMotors) {
            backLeft.drive(backLeftState);
        }

    }

    public boolean zeroModules() {
        boolean fl = frontLeft.resetSpinMotor();
        boolean fr = backLeft.resetSpinMotor();
        boolean br = backRight.resetSpinMotor();
        boolean bl = backLeft.resetSpinMotor();
        return fl && fr && br && bl;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}
