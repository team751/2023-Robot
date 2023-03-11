package frc.robot.commands;

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

public class Autonomous extends CommandBase {
  private final SwerveDrive swerb;
  private final Odometry navX2;
  //private ComplementaryFilter filteredAngles;
  private final Limelight limelight;
  private double[] robotXTargets;
  private double[] robotYTargets;
  private int targetNumber;
  private double robotXPoseFieldSpace;
  private double robotYPoseFieldSpace;
  private double robotXVelocityFieldSpace;
  private double robotYVelocityFieldSpace;
  private double robotXRotation;
  private double robotRotation;
  private double fieldSpaceXActiveVelocity;
  private double fieldSpaceYActiveVelocity;
  private double vx;
  private double vy;
  private final PIDController distanceController;
  private double deltaTime;
  private double olderTime;
  private final SlewRateLimiter vxlimiter;
  private final SlewRateLimiter vylimiter;
  private final SlewRateLimiter rylimiter;
  private ChassisSpeeds encoderChassisVelocity;

  /**
   * Creates a new AutonAutoLevel.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Autonomous(SwerveDrive subsystem, Limelight limelight, Odometry navX2) {
    this.limelight = limelight;
    this.navX2 = navX2;
    swerb = subsystem;
    addRequirements(swerb);
    addRequirements(limelight);
    rylimiter = new SlewRateLimiter(4);
    encoderChassisVelocity = swerb.robotPosEncoder();
    robotRotation = 0;
    targetNumber = 0;
    vxlimiter = new SlewRateLimiter(4);
    vylimiter = new SlewRateLimiter(4);
    deltaTime = 0;
    olderTime = 0;
    robotXTargets = Constants.autonTargetXPose;
    robotYTargets = Constants.autonTargetYPose;
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
    double[] values = limelight.getValues();
    if (values != null && values.length >= 3) {
        robotXPoseFieldSpace = values[0];
        robotYPoseFieldSpace = values[1];
        robotXRotation = values[3];
        robotRotation = values[4];
    } else {
        robotXPoseFieldSpace = robotXTargets[targetNumber];
        robotYPoseFieldSpace = robotYTargets[targetNumber];
    }
    // if robot is close to the target go to the next target
    if(Math.abs(robotXPoseFieldSpace - robotXTargets[targetNumber]) < 0.1 && Math.abs(robotYPoseFieldSpace - robotYTargets[targetNumber]) < 0.1) {
        targetNumber+=1;
    }
    //field space robot velocity
    fieldSpaceXActiveVelocity = distanceController.calculate(robotXPoseFieldSpace, robotXTargets[targetNumber]) * Constants.maxDriveSpeed / 2;
    fieldSpaceYActiveVelocity = distanceController.calculate(robotYPoseFieldSpace, robotYTargets[targetNumber]) * Constants.maxDriveSpeed / 2;
    //robot space robot velocity please DO NOT use the one in drive, as it only works if the gyro is aligned with field directions
    vx = fieldSpaceXActiveVelocity * Math.cos(Units.degreesToRadians(robotRotation)) + fieldSpaceYActiveVelocity * Math.sin(Units.degreesToRadians(robotRotation));
    vy = fieldSpaceXActiveVelocity * Math.sin(Units.degreesToRadians(robotRotation)) + fieldSpaceYActiveVelocity * Math.cos(Units.degreesToRadians(robotRotation));
    //align robot to field space in rotation
    double ry = distanceController.calculate(Units.degreesToRadians(robotRotation), 0);
    //execute the commands
    vx = vxlimiter.calculate(vx);
    vy = vylimiter.calculate(vy);
    ry = rylimiter.calculate(ry);

    //swerb.drive(vx, vy, ry,false);


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

