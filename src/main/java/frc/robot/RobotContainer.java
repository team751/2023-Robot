// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.subsystems.camera.Limelight;
import frc.robot.subsystems.gyro.ComplementaryFilter;
import frc.robot.commands.*;
import frc.robot.commands.testcommands.SwerveDriveTest;
import frc.robot.subsystems.gyro.NavX2;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // PRODUCTION COMMANDS
  private final SwerveModule frontLeftModule = new SwerveModule(Constants.SwerveModuleConfig.FRONT_LEFT);
  private final SwerveModule backLeftModule = new SwerveModule(Constants.SwerveModuleConfig.BACK_LEFT);
  private final SwerveModule backRightModule = new SwerveModule(Constants.SwerveModuleConfig.BACK_RIGHT);
  private final SwerveModule frontRightModule = new SwerveModule(Constants.SwerveModuleConfig.FRONT_RIGHT);
  private final Limelight limelight = new Limelight();
  private final ComplementaryFilter filteredAngles = new ComplementaryFilter();
  private final NavX2 navX2 = new NavX2();

  private final SwerveDrive swerve = new SwerveDrive(
      frontLeftModule, frontRightModule, backLeftModule, backRightModule);

  private final SwerveDriveCommand m_teleopCommand = new SwerveDriveCommand(swerve, limelight, filteredAngles, navX2);
  private final FollowAprilTag autonCommand = new FollowAprilTag(swerve, limelight, filteredAngles, navX2);

  //TESTING COMMANDS
  private final SwerveDriveTest m_testCommand = new SwerveDriveTest(backRightModule);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An FollowAprilTag will run in autonomous
    return autonCommand;
  }

  public Command getTeleopCommand() {
    // An SwerveDriveCommand will run in autonomous
    return m_teleopCommand;
  }

  public Command getTestCommand() {
    return m_testCommand;
  }
}
