package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Function;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.subsystems.absencoder.REVThroughBoreEncoder;
import frc.robot.subsystems.switches.ReedSwitch;

public class SwerveModule extends SubsystemBase {

  // Motors
  public final CANSparkMax driveMotor;
  public final CANSparkMax spinMotor;
  // Motor encoders
  public final RelativeEncoder spinEncoder;
  public final RelativeEncoder driveEncoder;
  public final REVThroughBoreEncoder absoluteEncoder;
  // Setpoint for absolute encoder
  public final double ABSOLUTE_ENCODER_OFFSET;
  public final double RELATIVE_ENCODER_OFFSET;
  // PID controllers for rotation
  public final PIDController rotationalPidController;
  public final PIDController zeroingPidController;

  // Auto-Zero helpers
  public ReedSwitch reedSwitch;
  public Debouncer zeroModuleDebouncer;
  public ZeroWheelsCommand zeroWheelsCommand;
  public boolean isZeroed;

  /** Creates a new SwerveDriveSubsystem. */
  public SwerveModule(int driveID, int spinID, int encoderID, double absoluteEncoderOffset, double relativeEncoderOffset, int reedSwitchPortID) {
    // PID controller for smoothly rotating the swerve module
    rotationalPidController = new PIDController(Constants.anglePIDDefaultValue, 0.5, 0.03);
    zeroingPidController = new PIDController(0.75, 0.55, 0.04);
    rotationalPidController.enableContinuousInput(-Math.PI, Math.PI);
    // Init motors
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    spinMotor = new CANSparkMax(spinID, MotorType.kBrushless);
    // Init encoders
    spinEncoder = spinMotor.getEncoder();
    driveEncoder = driveMotor.getEncoder();
    // converts encoder position (0 -> 1) to the current module angle
    spinEncoder.setPositionConversionFactor(2 * Math.PI / Constants.gearRatioSpin);
    this.ABSOLUTE_ENCODER_OFFSET = absoluteEncoderOffset;
    this.RELATIVE_ENCODER_OFFSET = relativeEncoderOffset;
    absoluteEncoder = new REVThroughBoreEncoder(encoderID);
    // Init reed switch
    reedSwitch = new ReedSwitch(reedSwitchPortID);
    // init debouncer
    zeroModuleDebouncer = new Debouncer(0.2);
    isZeroed = false;
   
    zeroWheelsCommand = new ZeroWheelsCommand(this);
  }

  public SwerveModule(Constants.SwerveModuleConfig moduleConfig) {
    this(moduleConfig.getDriveID(), moduleConfig.getSpinID(), moduleConfig.getEncoderID(),
        moduleConfig.getAbosoluteEncoderOffset(),moduleConfig.getRelativeEncoderOffset(), moduleConfig.getReedSwitchID());
    this.setName(moduleConfig.name());
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean(getName() + " reed switch", reedSwitch.get());
    debugPutValues();
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
    return new Rotation2d(spinEncoder.getPosition());
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
    //TODO: get values closer to theoretical max
    double motorSpeed = state.speedMetersPerSecond / Constants.maxDriveSpeed / 1.5;
    driveMotor.set(motorSpeed);
    return driveMotor.getEncoder().getVelocity();
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(
      // TODO: Unchecked Conversion
      this.driveEncoder.getPosition() * Constants.wheelCircumference * Constants.gearRatioDrive,
      getCurrentRotation2d()
    );
  }

  public void spinWithoutDriving(SwerveModuleState state){
    double angleSetpoint = state.angle.getRadians();
    setAngle(angleSetpoint);
  }

  public ZeroWheelsCommand getZeroCommand(){
    return zeroWheelsCommand;
  }

  public boolean isZeroing(){
    return zeroWheelsCommand.isScheduled();
  }

  public void debugPutValues() {
    SmartDashboard.putNumber(this.getName() + " Absolute Encoder Angle (rad) ", absoluteEncoder.getAbsolutePositionRadians());
    SmartDashboard.putNumber(this.getName() + " Wheel Angle ", spinEncoder.getPosition());
    SmartDashboard.putNumber(this.getName() + " Angle (rad) ", getCurrentAngleRadians());
    SmartDashboard.putNumber(this.getName() + " Drive Encoder Velocity", getDriveEncoderVelocity());
    reedSwitch.debugPutValues();
  }

  public void stop() {
    driveMotor.set(0);
    spinMotor.set(0);
  }

  public double getDriveEncoderVelocity(){
    return driveEncoder.getVelocity() * Constants.maxDriveSpeed;
  }

}
