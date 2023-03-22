package frc.robot.commands;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
  private double[] robotXStatesFieldSpace;
  private double[] robotYStatesFieldSpace;
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
  public class PVA {
    public double p;
    public double v;
    public double a;
    public double position;
    public double velocity;
    public double acceleration;
    public PVA(double p, double v, double a) {
      this.p = p;
      this.v = v;
      this.a = a;
      this.position = p;
      this.velocity = v;
      this.acceleration = a;
    }
  }

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
    robotXStatesFieldSpace = new double[]{0,0,0}; // to simplify the 3 states for each axis, his array contains the acceleration, velocity and displacement of the robot in the x axis
    robotYStatesFieldSpace = new double[]{0,0,0};

    distanceController = new PIDController(Constants.autonDistancePIDDefaultValue,
            Constants.autonDistancePIDIntegralValue, Constants.autonDistancePIDDerivativeValue);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // past velocity and past acceleration can not be calculated with the sensor data as that removes the filtering so it must be identified from the previous iteration
  private double linearInterpolation(double pastDisplacement, double pastVelocity, double pastAcc, double sensorDisplacement) {
    double predictedDisplacement = pastDisplacement + pastVelocity * deltaTime+ 0.5 * pastAcc * deltaTime * deltaTime;
    return predictedDisplacement*(1-Constants.pastBias)+Constants.pastBias*sensorDisplacement;
  }

  private double[] updateInterpolation(double[] initialStates, double sensorData){
    double[] finalStates = new double[3];
    finalStates[2] = linearInterpolation(initialStates[2], initialStates[1], initialStates[0], sensorData);
    finalStates[1] = (finalStates[2]-initialStates[2])/deltaTime;
    finalStates[0] = (finalStates[1]-initialStates[1])/deltaTime;
    return finalStates;
  }




  // Called every time the scheduler runs while the command is scheduled.
  @Override
  @Deprecated
  public void execute() {
    deltaTime = (System.currentTimeMillis() - olderTime) / 1000;
    olderTime = System.currentTimeMillis();
    navX2.debugPutValues();
    limelight.debugDisplayValues();
    double[] values = limelight.getValuesAsArray();
    if (values != null && values.length >= 3) {
        robotXStatesFieldSpace[2] = values[0];
        robotYStatesFieldSpace[2] = values[1];
        robotRotation = values[4];
    } else {
        robotXStatesFieldSpace[2] = robotXTargets[targetNumber];
        robotYStatesFieldSpace[2] = robotYTargets[targetNumber];
        robotRotation = 0;
    }
    

    // if robot is close to the target go to the next target
    if(Math.abs(robotXStatesFieldSpace[2] - robotXTargets[targetNumber]) < 0.1 && Math.abs(robotYStatesFieldSpace[2] - robotYTargets[targetNumber]) < 0.1 && targetNumber < robotXTargets.length - 1) {
        targetNumber+=1;
    }
    //field space robot velocity
    fieldSpaceXActiveVelocity = distanceController.calculate(robotXStatesFieldSpace[2], robotXTargets[targetNumber]) * Constants.maxDriveSpeed / 2;
    fieldSpaceYActiveVelocity = distanceController.calculate(robotXStatesFieldSpace[2], robotYTargets[targetNumber]) * Constants.maxDriveSpeed / 2;
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

