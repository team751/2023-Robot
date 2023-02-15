package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.subsystems.absencoder.AbsoluteEncoder;
import frc.robot.subsystems.switches.ReedSwitch;

public class SwerveModule extends SubsystemBase {

  // Motors
  private final CANSparkMax driveMotor;
  private final CANSparkMax spinMotor;
  // Motor encoders
  private final RelativeEncoder encoder;
  private final AbsoluteEncoder absoluteEncoder;
  // Setpoint for absolute encoder
  private final double ENCODER_OFFSET;
  // PID controllers for rotation
  private final PIDController rotationalPidController;

  // Instance variables for returning values
  private ReedSwitch reedSwitch;

  /** Creates a new SwerveDriveSubsystem. */
  public SwerveModule(int driveID, int spinID, int encoderID, double absoluteEncoderOffset, int reedSwitchPortID) {
    // PID controller for smoothly rotating the swerve module
    rotationalPidController = new PIDController(Constants.anglePIDDefaultValue, 0, 0);
    rotationalPidController.enableContinuousInput(-Math.PI, Math.PI);
    // Init motors
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    spinMotor = new CANSparkMax(spinID, MotorType.kBrushless);
    // Init encoders
    encoder = spinMotor.getEncoder();
    // converts encoder position (0 -> 1) to the current module angle
    encoder.setPositionConversionFactor(2 * Math.PI / Constants.gearRatioSpin);
    ENCODER_OFFSET = absoluteEncoderOffset;
    absoluteEncoder = new AbsoluteEncoder(encoderID);
    // Init reed switch
    reedSwitch = new ReedSwitch(reedSwitchPortID);
  }

  public SwerveModule(Constants.SwerveModuleConfig moduleConfig) {
    this(moduleConfig.getDriveID(), moduleConfig.getSpinID(), moduleConfig.getEncoderID(),
        moduleConfig.getEncoderOffset(), moduleConfig.getReedSwitchID());
    this.setName(moduleConfig.name());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setSpeed(double speedMetersPerSecond) {
    double motorSpeed = speedMetersPerSecond / Constants.maxDriveSpeed / Constants.driveMotorMaxSpeedRatio;
    driveMotor.set(motorSpeed);
  }

  public void setSpinSpeed(double speed) {
    spinMotor.set(speed);
  }

  public Rotation2d getCurrentRotation2d() {
    return new Rotation2d(encoder.getPosition());
  }

  public double getCurrentAngleRadians() {
    return getCurrentRotation2d().getRadians();
  }

  public double setAngle(double setpointRadians) {
    double motorSpeed = rotationalPidController.calculate(getCurrentAngleRadians(), setpointRadians);
    double normalizedMotorSpinSpeed = (motorSpeed / Constants.spinMotorMaxSpeedMetersPerSecond)
        * Constants.gearRatioSpin;
    spinMotor.set(normalizedMotorSpinSpeed);
    return normalizedMotorSpinSpeed;
  }

  public double drive(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getCurrentRotation2d());
    // setting the angle
    double angleSetpoint = state.angle.getRadians();
    setAngle(angleSetpoint);

    // set the drive speed to 1/3 of its theoretical max
    double motorSpeed = state.speedMetersPerSecond / Constants.maxDriveSpeed / 3;
    driveMotor.set(motorSpeed);
    return driveMotor.getEncoder().getVelocity();
  }

  public boolean resetSpinMotor() {
    double motorSpeed = rotationalPidController.calculate(absoluteEncoder.getPositionRadians(), ENCODER_OFFSET);
    double normalizedMotorSpinSpeed = (motorSpeed / Constants.spinMotorMaxSpeedMetersPerSecond)
        * Constants.gearRatioSpin;
    spinMotor.set(normalizedMotorSpinSpeed);
    if (Math.abs(absoluteEncoder.getPositionRadians() - ENCODER_OFFSET) < 0.01) {
      encoder.setPosition(0);
      // if the reed switch is not triggered (meaning the module is opposite where it
      // should be) then invert the motor's direction
      if (!reedSwitch.get()) {
        driveMotor.setInverted(true);
      } else {
        driveMotor.setInverted(false);
      }
      return true;
    }
    return false;
  }

  public void debugPutValues() {
    SmartDashboard.putNumber(this.getName() + " Absolute Encoder Angle", absoluteEncoder.getPositionRadians() * 360);
    SmartDashboard.putNumber(this.getName() + " Relative Encoder Angle", encoder.getPosition() * 360);
    SmartDashboard.putNumber(this.getName() + " Angle (Degrees)", Units.radiansToDegrees(getCurrentAngleRadians()));
    SmartDashboard.putNumber(this.getName() + " Encoder Velocity", driveMotor.getEncoder().getVelocity());
    reedSwitch.debugPutValues();
  }

  public void stop() {
    driveMotor.set(0);
    spinMotor.set(0);
  }
}
