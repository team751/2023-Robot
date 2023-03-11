package frc.robot.commands.testcommands;

import frc.robot.subsystems.drivetrain.SwerveDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotationAndSmoothingTest extends CommandBase {
  private final SwerveDrive swerveDrive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotationAndSmoothingTest(SwerveDrive subsystem) {
    this.swerveDrive = subsystem;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
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
    return false;
  }
}

