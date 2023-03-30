package frc.robot.subsystems.drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveModule;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroWheelsCommand extends CommandBase {
  private final SwerveModule module;
  private double startTime;
  /**
   * Creates a new zeroWheelsCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ZeroWheelsCommand(SwerveModule subsystem) {
    this.module = subsystem;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.module.stop();
    this.startTime = System.currentTimeMillis();
  }

  boolean hasRun = false;
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
    module.spinEncoder.setPosition(-module.RELATIVE_ENCODER_OFFSET);
    if (!module.reedSwitch.get()) {
        module.driveMotor.setInverted(true);
    } else {
        module.driveMotor.setInverted(false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceFromZero = Math.abs(module.absoluteEncoder.getAbsolutePositionRadians() - module.ABSOLUTE_ENCODER_OFFSET);
    return module.zeroModuleDebouncer.calculate(distanceFromZero < 0.07) || System.currentTimeMillis() - startTime > 5000;
  }
}

