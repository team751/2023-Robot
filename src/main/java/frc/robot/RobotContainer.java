// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.introspect.WithMember;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.absencoder.*;
import frc.robot.subsystems.gyro.*;
import frc.robot.subsystems.camera.*;
import frc.robot.subsystems.switches.ReedSwitch;
import frc.robot.subsystems.thebelt.TheBelt;
import frc.robot.subsystems.wheelyarm.WheelyArm;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.camera.Limelight;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.gyro.ComplementaryFilter;
import frc.robot.commands.*;
import frc.robot.commands.testcommands.FollowAprilTag;
import frc.robot.commands.testcommands.SwerveDriveTest;
import frc.robot.subsystems.gyro.Odometry;
import frc.robot.commands.TheBeltCommand;

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
  // SUBSYSTEMS
  private final SwerveModule frontLeftModule = new SwerveModule(Constants.SwerveModuleConfig.FRONT_LEFT);
  private final SwerveModule backLeftModule = new SwerveModule(Constants.SwerveModuleConfig.BACK_LEFT);
  private final SwerveModule backRightModule = new SwerveModule(Constants.SwerveModuleConfig.BACK_RIGHT);
  private final SwerveModule frontRightModule = new SwerveModule(Constants.SwerveModuleConfig.FRONT_RIGHT);
  private final TheBelt theBelt = new TheBelt(Constants.beltMotorPort,Constants.beltFanPort1,Constants.beltFanPort2);//TODO: PUT IN CONSTANTS
 // private final WheelyArm wheelyArm = new WheelyArm(6,7);

  // PRODUCTION COMMANDS
  private final Limelight limelight = new Limelight();
  private final Odometry navX2 = Odometry.getInstance();

  private final SwerveDrive swerve = new SwerveDrive(
      frontLeftModule, 
      frontRightModule, 
      backLeftModule, 
      backRightModule);
  private final AutoLevel autoLevel = new AutoLevel(navX2, swerve);
  private final Drive drive = new Drive(swerve, limelight, navX2);
  private final TheBeltCommand beltCommand = new TheBeltCommand(theBelt);
  //private final WheelyArmCommand wheelyArmCommand = new WheelyArmCommand(wheelyArm);

  private final AutonSimple m_autonCommand = new AutonSimple(swerve,beltCommand);

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
    Constants.driverController.a().toggleOnTrue(autoLevel.unless(() -> swerve.isZeroing()));
    Constants.driverController.b().onTrue(Commands.runOnce(swerve::zeroModules).unless(autoLevel::isScheduled));
    Constants.driverController.x().whileTrue(beltCommand);
   // Constants.driverController.y().whileTrue(wheelyArmCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A FollowAprilTag will run in autonomous
    return m_autonCommand;
  }

  public Command getTeleopCommand() {
    // A Drive will run in teleop
    return drive;
  }

  public Command getTestCommand() {
    return m_testCommand;
  }
}
