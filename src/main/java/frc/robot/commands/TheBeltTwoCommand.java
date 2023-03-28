package frc.robot.commands;

import frc.robot.subsystems.thebelt.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TheBeltTwoCommand extends CommandBase {
  private final TheBelt belt;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TheBeltTwoCommand(TheBelt subsystem) {
    belt = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    belt.run(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    belt.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

