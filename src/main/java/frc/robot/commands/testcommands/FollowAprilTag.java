package frc.robot.commands.testcommands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.camera.Limelight;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
//import frc.robot.subsystems.gyro.ComplementaryFilter;
import frc.robot.subsystems.gyro.Odometry;

public class FollowAprilTag extends CommandBase {
    private final SwerveDrive swerb;
    private final Odometry navX2;
    //private ComplementaryFilter filteredAngles;
    private final Limelight limelight;
    private double xTarget;
    private double yTarget;
    private double robotXPos;
    private double robotYPos;
    private double vx;
    private double vy;
    private final PIDController distanceController;
    private double deltaTime;
    private double olderTime;
    private final SlewRateLimiter vxlimiter;
    private final SlewRateLimiter vylimiter;
    private final SlewRateLimiter rylimiter;
    private double yRotation;


    private ChassisSpeeds encoderChassisVelocity;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */

    @Deprecated
    public FollowAprilTag(SwerveDrive subsystem, Limelight limelight, Odometry navX2) {
        rylimiter = new SlewRateLimiter(4);
        yRotation = 0;
        this.limelight = limelight;
        this.navX2 = navX2;
        swerb = subsystem;
        encoderChassisVelocity = swerb.robotPosEncoder();
        addRequirements(swerb);
        addRequirements(limelight);
        xTarget = 3;
        yTarget = 0;
        robotXPos = xTarget;
        robotYPos = yTarget;
        vxlimiter = new SlewRateLimiter(4);
        vylimiter = new SlewRateLimiter(4);
        deltaTime = 0;
        olderTime = 0;
        distanceController = new PIDController(Constants.autonDistancePIDDefaultValue,
                Constants.autonDistancePIDIntegralValue, Constants.autonDistancePIDDerivativeValue);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        deltaTime = (System.currentTimeMillis() - olderTime) / 1000;
        olderTime = System.currentTimeMillis();
        navX2.debugPutValues();
        limelight.debugDisplayValues();
        double[] values = limelight.getValuesAsArray();
        if (values != null && values.length >= 3) {
            robotXPos = values[2];
            robotYPos = values[0];
            yRotation = values[4];
        } else {
            robotXPos = xTarget;
            robotYPos = yTarget;
        }
        vx = distanceController.calculate(robotXPos, xTarget) * Constants.maxDriveSpeed / 2;
        vy = distanceController.calculate(robotYPos, yTarget) * Constants.maxDriveSpeed / 2;
        double ry = distanceController.calculate(Units.degreesToRadians(yRotation), 0);
        vx = vxlimiter.calculate(vx);
        vy = vylimiter.calculate(vy);
        ry = rylimiter.calculate(ry);

        //filteredAngles.debugAngle();
        swerb.drive(vx, vy, ry,false);
        SmartDashboard.putNumber("encoderbased vx", encoderChassisVelocity.vxMetersPerSecond);
        SmartDashboard.putNumber("encoderbased vy", encoderChassisVelocity.vyMetersPerSecond);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
