package frc.robot.commands;

import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyro.Odometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonSimple extends CommandBase {
  private final SwerveDrive m_subsystem;
  private final TheBeltCommand beltCommand;
  private final AutoLevel autoLevel;
  private final Odometry navx2;
  boolean hasRun;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonSimple(SwerveDrive subsystem, TheBeltCommand command, AutoLevel autoLevel, Odometry navx2) {
    m_subsystem = subsystem;
    beltCommand = command;
    this.autoLevel = autoLevel;
    this.navx2 = navx2;


    SmartDashboard.putBoolean("Engage", true);
    SmartDashboard.putBoolean("Mobility",true);

    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasRun = false;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //Drive fwd at 5 m/s
    if(SmartDashboard.getBoolean("Mobility",true) && !hasRun){
    try{
      beltCommand.execute();
      Thread.sleep(2000);
      beltCommand.end(true);
      long start = System.currentTimeMillis();
      while(System.currentTimeMillis() - start < 2300){
        m_subsystem.drive(0,1.5,0,false);
      }
      SmartDashboard.putNumber("Engage angle", navx2.getYAngle()+88);
      while(Math.abs(navx2.getYAngle()+88) > 3){
        m_subsystem.drive(0,1.5,0,false);
        
      }
      m_subsystem.stop();
      m_subsystem.crossWheels();
      start = System.currentTimeMillis();
      if(SmartDashboard.getBoolean("Engage", true)){
        while(System.currentTimeMillis() - start < 9000){
        autoLevel.execute();
        }
      }
      m_subsystem.stop();
      hasRun = true;
    }catch(InterruptedException e){
      e.printStackTrace();
    }
  }
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
