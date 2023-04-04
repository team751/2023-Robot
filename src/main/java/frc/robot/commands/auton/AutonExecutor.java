package frc.robot.commands.auton;

import frc.robot.commands.auton.paths.Engage;
import frc.robot.commands.auton.paths.Mobility;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonExecutor extends CommandBase {
  private final SwerveDrive swerve;
  private final AHRS navX2;
  private final Engage engageCommand;
  private final Mobility mobilityCommand;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonExecutor(SwerveDrive subsystem, AHRS navX2) {
    this.swerve = subsystem;
    addRequirements(subsystem);
    this.navX2 = navX2;
    engageCommand = new Engage(swerve, this.navX2);
    mobilityCommand = new Mobility(swerve, this.navX2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Current Mode", "Auton");
    if(SmartDashboard.getBoolean("Engage", false)){
      engageCommand.schedule();
    }else{
      mobilityCommand.schedule();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // janky if we need to add more paths, but should work
    mobilityCommand.cancel();
    engageCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // janky but should work
    return mobilityCommand.isFinished() || engageCommand.isFinished();
  }
}
