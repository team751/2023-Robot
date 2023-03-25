package frc.robot.commands;

import frc.robot.subsystems.wheelyarm.WheelyArm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WheelyArmCommand extends CommandBase {
  private final WheelyArm m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WheelyArmCommand(WheelyArm subsystem) {
    m_subsystem = subsystem;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.run(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

