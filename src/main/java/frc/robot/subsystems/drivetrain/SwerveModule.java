package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private final CANSparkMax driveMotor;
  private final CANSparkMax spinMotor;
  // Motor encoders
  private final RelativeEncoder spinEncoder;
  private final RelativeEncoder driveEncoder;
  private final REVThroughBoreEncoder absoluteEncoder;
  // Setpoint for absolute encoder
  private final double ABSOLUTE_ENCODER_OFFSET;
  private final double RELATIVE_ENCODER_OFFSET;
  // PID controllers for rotation
  private final PIDController rotationalPidController;
  private final PIDController zeroingPidController;

  // Auto-Zero helpers
  private ReedSwitch reedSwitch;
  private Debouncer zeroModuleDebouncer;
  private Debouncer isFinishedDebouncer;
  private FunctionalCommand zeroWheelsCommand;

  /** Creates a new SwerveDriveSubsystem. */
  public SwerveModule(int driveID, int spinID, int encoderID, double absoluteEncoderOffset, double relativeEncoderOffset, int reedSwitchPortID) {
    // PID controller for smoothly rotating the swerve module
    rotationalPidController = new PIDController(Constants.anglePIDDefaultValue, 0.4, 0.01);
    zeroingPidController = new PIDController(Constants.anglePIDDefaultValue, 0.4, 0.01);
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
    isFinishedDebouncer = new Debouncer(0.2);
   
    zeroWheelsCommand = new FunctionalCommand(()->SmartDashboard.putString("CurrentMode","Zeroing"), this::resetSpinMotor, this::setRelativeEncodersToZero, this::isZeroed);
  }

  public SwerveModule(Constants.SwerveModuleConfig moduleConfig) {
    this(moduleConfig.getDriveID(), moduleConfig.getSpinID(), moduleConfig.getEncoderID(),
        moduleConfig.getAbosoluteEncoderOffset(),moduleConfig.getRelativeEncoderOffset(), moduleConfig.getReedSwitchID());
    this.setName(moduleConfig.name());
    SmartDashboard.putBoolean(getName() + "invert?", false);
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
    driveMotor.setInverted(SmartDashboard.getBoolean(getName() + "invert?", false));
    // setting the angle
    double angleSetpoint = state.angle.getRadians();
    setAngle(angleSetpoint);

    // set the drive speed to 1/3 of its theoretical max
    //TODO: get values closer to theoretical max
    double motorSpeed = state.speedMetersPerSecond / Constants.maxDriveSpeed / 1.2;
    driveMotor.set(motorSpeed);
    return driveMotor.getEncoder().getVelocity();
  }

  public void spinWithoutDriving(SwerveModuleState state){
    double angleSetpoint = state.angle.getRadians();
    setAngle(angleSetpoint);
  }

  private void resetSpinMotor() {
    driveMotor.set(0);
    double motorSpeed = zeroingPidController.calculate(absoluteEncoder.getAbsolutePositionRadians(), ABSOLUTE_ENCODER_OFFSET);
    double normalizedMotorSpinSpeed = (motorSpeed / Constants.spinMotorMaxSpeedMetersPerSecond)
        * Constants.gearRatioSpin;
    spinMotor.set(normalizedMotorSpinSpeed);
    if (isZeroedSingleDebounce()) {
      // if the reed switch is not triggered (meaning the module is opposite where it
      // should be) then invert the motor's direction
      spinEncoder.setPosition(-RELATIVE_ENCODER_OFFSET);
      Thread t = new Thread(() -> {
        try {
          Thread.sleep(100);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
        if (!reedSwitch.get()) {
          driveMotor.setInverted(true);
        } else {
          driveMotor.setInverted(false);
        }
      });
      t.start();
    }
  }

  private boolean isZeroed(){
      return isFinishedDebouncer.calculate(isZeroedSingleDebounce());
  }

  private boolean isZeroedSingleDebounce(){
     return zeroModuleDebouncer.calculate(Math.abs(absoluteEncoder.getAbsolutePositionRadians() - ABSOLUTE_ENCODER_OFFSET) < 0.1);
  }

  
  private void setRelativeEncodersToZero(boolean interrupted){
    if (!reedSwitch.get()) {
      driveMotor.setInverted(true);
    } else {
      driveMotor.setInverted(false);
    }
  }

  public FunctionalCommand getZeroCommand(){
    return zeroWheelsCommand;
  }
  public boolean isZeroing(){
    return zeroWheelsCommand.isScheduled();
  }

  public void debugPutValues() {
    SmartDashboard.putNumber(this.getName() + " Absolute Encoder Angle (rad) ", absoluteEncoder.getAbsolutePositionRadians());
    SmartDashboard.putNumber(this.getName() + " Wheel Angle", spinEncoder.getPosition());
    SmartDashboard.putNumber(this.getName() + " Angle (rad)", getCurrentAngleRadians());
    SmartDashboard.putNumber(this.getName() + " Encoder Velocity", driveMotor.getEncoder().getVelocity());
    reedSwitch.debugPutValues();
  }

  public void stop() {
    driveMotor.set(0);
    spinMotor.set(0);
  }

  public double getDriveEncoderVelocity(){
    return driveEncoder.getVelocity()*Constants.maxDriveSpeed;
  }


}
