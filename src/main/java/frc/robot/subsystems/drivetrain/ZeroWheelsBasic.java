package frc.robot.subsystems.drivetrain;

import frc.robot.Constants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroWheelsBasic extends CommandBase {
  private final SwerveModule module;
  private final Debouncer reedSwitchDebouncer;
  private double startTime;
  /**
   * Creates a new zeroWheelsCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ZeroWheelsBasic(SwerveModule subsystem) {
    this.module = subsystem;
    // Should help remove false positives, and get the zeroed spot closer to centered
    this.reedSwitchDebouncer = new Debouncer(0.05);
    this.startTime = System.currentTimeMillis();
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.module.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double motorSpeed = module.zeroingPidController.calculate(module.absoluteEncoder.getAbsolutePositionRadians(), module.ABSOLUTE_ENCODER_OFFSET);
    double normalizedMotorSpinSpeed = (motorSpeed / Constants.spinMotorMaxSpeedMetersPerSecond) * Constants.gearRatioSpin;
    module.setSpinSpeed(normalizedMotorSpinSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) return;
    module.setSpinSpeed(0);
    // not sure if this works yet, but it *should* account for 
    // the difference between the actual position of the wheel and it's theoretical position
    double a = module.ABSOLUTE_ENCODER_OFFSET - module.absoluteEncoder.getAbsolutePositionRadians();
    module.spinEncoder.setPosition(-module.RELATIVE_ENCODER_OFFSET + a/2);
    module.driveMotor.setInverted(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // timeout of 5 seconds
    if(System.currentTimeMillis() - startTime > 5000) return true;
    return reedSwitchDebouncer.calculate(module.reedSwitch.get());
  }
}

