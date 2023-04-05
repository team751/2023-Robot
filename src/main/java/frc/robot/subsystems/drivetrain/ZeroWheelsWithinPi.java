package frc.robot.subsystems.drivetrain;

import frc.robot.subsystems.drivetrain.SwerveModule;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroWheelsWithinPi extends CommandBase {
  private final SwerveModule module;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ZeroWheelsWithinPi(SwerveModule subsystem) {
    module = subsystem;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    module.spinEncoder.setPosition(-(module.RELATIVE_ENCODER_OFFSET - (module.absoluteEncoder.getAbsolutePositionRadians() - module.ABSOLUTE_ENCODER_OFFSET)/2));
    System.out.println("zeroed within pi");
    module.driveMotor.setInverted(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
