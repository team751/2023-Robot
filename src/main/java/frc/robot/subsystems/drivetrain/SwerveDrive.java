package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.gyro.Odometry;

// Subsystem controlling all four individual swerve modules
public class SwerveDrive extends SubsystemBase {
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;

    private SwerveDriveKinematicsConstraint velocityNormalizer;
    private SwerveDriveKinematics kinematics;
    private Odometry navX2;

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
        velocityNormalizer = new SwerveDriveKinematicsConstraint(kinematics, Constants.maxDriveSpeed);
        SmartDashboard.putBoolean("Front Right Motor", true);
        SmartDashboard.putBoolean("Front Left Motor", true);
        SmartDashboard.putBoolean("Back Right Motor", true);
        SmartDashboard.putBoolean("Back Left Motor", true);
        SmartDashboard.putBoolean("Disable All Motors", false);
    }

    public void drive(double vx,double vy, double rotationRadiansPerSecond,boolean fieldCentric){
        drive(vx,vy,rotationRadiansPerSecond,fieldCentric,false);
    }

    private void drive(double vx, double vy, double rotationRadiansPerSecond,boolean fieldCentric,boolean spinWithoutDriving) {
        // Joystick values to a speed vector
        // Convert speed vector and rotation to module speeds
        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, rotationRadiansPerSecond);
        if(fieldCentric == true){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rotationRadiansPerSecond, navX2.getHeading());
        }
        // Unpack module speeds
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveModuleState frontLeftState = states[0];
        SwerveModuleState frontRightState = states[1];
        SwerveModuleState backLeftState = states[2];
        SwerveModuleState backRightState = states[3];

        // stops modules from resetting their position if the joystick is not being used
        if (Math.sqrt(vx * vx + vy * vy) < 0.05 && Math.abs(rotationRadiansPerSecond) < 0.05) {
            this.stop();
            return;
        }

        // Override to enable all motors at once on SmartDashboard, on by default
        boolean disableAllMotors = SmartDashboard.getBoolean("Disable All Motors", false);
        if(disableAllMotors){
            this.stop();
            return;
        }

        /* Set swerve module speeds and rotations, also put them to smartdashboard */
        if (moduleEnabled("Front Right")) {
            // Actual Driving
            if(!spinWithoutDriving) frontRight.drive(frontRightState);
            else frontRight.spinWithoutDriving(frontRightState);
        }
        if (moduleEnabled("Front Left")) {
            if(!spinWithoutDriving) {
                frontLeft.drive(frontLeftState);
            } else {
                frontLeft.spinWithoutDriving(frontLeftState);
            }
        }
        if (moduleEnabled("Back Right")) {
            if(!spinWithoutDriving) {
                backRight.drive(backRightState);
            } else {
                backRight.spinWithoutDriving(backRightState);
            }
        }
        if (moduleEnabled("Back Left")) {
            if(!spinWithoutDriving) {
                backLeft.drive(backLeftState);
            } else {
                backLeft.spinWithoutDriving(backLeftState);
            }
        }
    }

    private boolean moduleEnabled(String moduleName){
        switch(moduleName){
            case "Front Left":
                return SmartDashboard.getBoolean("Front Left Motor", true);
            case "Front Right":
                return SmartDashboard.getBoolean("Front Right Motor", true);
            case "Back Left":
                return SmartDashboard.getBoolean("Back Left Motor", true);
            case "Back Right":
                return SmartDashboard.getBoolean("Back Right Motor", true);
            default:
                return false;
        }
    }

    public void zeroModules() {
        SmartDashboard.putString("Current Mode", "Zeroing");
        frontLeft.getZeroCommand().schedule();
        frontRight.getZeroCommand().schedule();
        backLeft.getZeroCommand().schedule();
        backRight.getZeroCommand().schedule();
    }

    public boolean isZeroing(){
        return frontLeft.isZeroing() || frontRight.isZeroing() || backLeft.isZeroing() || backRight.isZeroing();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void crossWheels(){
        frontLeft.setSpeed(0);
        frontRight.setSpeed(0);
        backLeft.setSpeed(0);
        backRight.setSpeed(0);

        frontLeft.setAngle(-Math.PI/2);
        frontRight.setAngle(Math.PI/2);
        backLeft.setAngle(Math.PI);
        backRight.setAngle(-Math.PI);
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    } 

    public void debugPutEncoderValues(){
        frontLeft.debugPutValues();
        frontRight.debugPutValues();
        backLeft.debugPutValues();
        backRight.debugPutValues();
    }

    public ChassisSpeeds robotPosEncoder(){
        SwerveModuleState frontLeftEncoderState = new SwerveModuleState(frontLeft.getDriveEncoderVelocity(), frontLeft.getCurrentRotation2d());
        SwerveModuleState frontRightEncoderState = new SwerveModuleState(frontRight.getDriveEncoderVelocity(), frontRight.getCurrentRotation2d());
        SwerveModuleState backLeftEncoderState = new SwerveModuleState(backLeft.getDriveEncoderVelocity(), backLeft.getCurrentRotation2d());
        SwerveModuleState backRightEncoderState = new SwerveModuleState(backRight.getDriveEncoderVelocity(), backRight.getCurrentRotation2d());
        return kinematics.toChassisSpeeds(frontLeftEncoderState, frontRightEncoderState, backLeftEncoderState, backRightEncoderState);
    }
}
