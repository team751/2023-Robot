package frc.robot.commands.auton.basic;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class RotateChassis extends CommandBase {
  private final SwerveDrive swerveDrive;
  private final AHRS navX2;
  private double startDistance;
  private final double goalDistance;
  private final double speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotateChassis(SwerveDrive subsystem, AHRS navX2, double distanceMeters, double speedMetersPerSecond) {
    this.swerveDrive = subsystem;
    this.navX2 = navX2;
    this.goalDistance = distanceMeters;
    this.speed = speedMetersPerSecond;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startDistance = navX2.getDisplacementX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.drive(0,0,speed,false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return navX2.getDisplacementX() - startDistance > goalDistance;
  }
}

