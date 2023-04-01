package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConfig;

// Subsystem controlling all four individual swerve modules
public class SwerveDrive extends SubsystemBase {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final SwerveDriveKinematicsConstraint velocityNormalizer;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final AHRS navX2;

    /** Creates a new ExampleSubsystem. */
    public SwerveDrive(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft,
            SwerveModule backRight, AHRS navX2) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.navX2 = navX2;

        kinematics = new SwerveDriveKinematics(
                Constants.frontLeftOffsetMeters,
                Constants.frontRightOffsetMeters,
                Constants.backLeftOffsetMeters,
                Constants.backRightOffsetMeters);
        velocityNormalizer = new SwerveDriveKinematicsConstraint(kinematics, Constants.maxDriveSpeed);
        this.odometry = new SwerveDriveOdometry(kinematics, navX2.getRotation2d(), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }, new Pose2d(0.0,0.0, new Rotation2d(0))
        );

        SmartDashboard.putBoolean("Front Right Motor", true);
        SmartDashboard.putBoolean("Front Left Motor", true);
        SmartDashboard.putBoolean("Back Right Motor", true);
        SmartDashboard.putBoolean("Back Left Motor", true);
        SmartDashboard.putBoolean("Disable All Motors", false);
    }

    public void drive(double vx, double vy, double rotationRadiansPerSecond,boolean fieldCentric) {
        // Joystick values to a speed vector
        // Convert speed vector and rotation to module speeds
        ChassisSpeeds speeds;
        if(fieldCentric){
            double yaw = navX2.getYaw();
            SmartDashboard.putNumber("navx yaw", yaw);
            // + 180 due to mounting location
            Rotation2d heading = new Rotation2d(Units.degreesToRadians(-navX2.getYaw() + 180));
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rotationRadiansPerSecond, heading);
        }else{
            speeds = new ChassisSpeeds(vx, vy, rotationRadiansPerSecond);
        }

        // Unpack module speeds
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveModuleState frontLeftState = states[0];
        SwerveModuleState frontRightState = states[1];
        SwerveModuleState backLeftState = states[2];
        SwerveModuleState backRightState = states[3];

        // stops modules from resetting their position if the joystick is not being used
        if (Math.sqrt(vx * vx + vy * vy) < 0.1 && Math.abs(rotationRadiansPerSecond) < 0.1) {
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
        if (moduleEnabled(SwerveModuleConfig.FRONT_LEFT)) {
            // Actual Driving
            frontRight.drive(frontRightState);
        }
        if (moduleEnabled(SwerveModuleConfig.FRONT_RIGHT)) {
            frontLeft.drive(frontLeftState);
        }
        if (moduleEnabled(SwerveModuleConfig.BACK_RIGHT)) {
            backRight.drive(backRightState);
        }
        if (moduleEnabled(SwerveModuleConfig.BACK_LEFT)) {
            backLeft.drive(backLeftState);
        }
    }

    private boolean moduleEnabled(SwerveModuleConfig moduleName){
        switch(moduleName){
            case FRONT_LEFT:
                return SmartDashboard.getBoolean("Front Left Motor", true);
            case FRONT_RIGHT:
                return SmartDashboard.getBoolean("Front Right Motor", true);
            case BACK_RIGHT:
                return SmartDashboard.getBoolean("Back Left Motor", true);
            case BACK_LEFT:
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
