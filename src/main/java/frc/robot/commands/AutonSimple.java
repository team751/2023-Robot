package frc.robot.commands;

import frc.robot.subsystems.drivetrain.SwerveDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonSimple extends CommandBase {
  private final SwerveDrive m_subsystem;
  private final TheBeltCommand beltCommand;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonSimple(SwerveDrive subsystem, TheBeltCommand command) {
    m_subsystem = subsystem;
    beltCommand = command;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try{
      beltCommand.execute();
      Thread.sleep(2000);
      beltCommand.end(true);
      long start = System.currentTimeMillis();
      while(System.currentTimeMillis() - start < 3100){
        m_subsystem.drive(0,1.5,0,false);
      }
      m_subsystem.stop();
    }catch(InterruptedException e){
      e.printStackTrace();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //Drive fwd at 5 m/s
      
      
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
