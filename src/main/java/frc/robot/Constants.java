// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

/* 
    ==========
    DRIVETRAIN 
    ==========
*/
    // Port values/offsets for swerve module initialization
    //TODO: Find correct encoder offsets
    private static final double halfpi = Math.PI/2;
    public enum SwerveModuleConfig {
        // redo relative
        FRONT_RIGHT(12, 13, 1, 3.106, 0.6264488101 + halfpi, 5),
        FRONT_LEFT(15, 14, 3, 3.132, 2.6927967072 + halfpi, 6),
        BACK_LEFT(16, 17, 2, 0.8400, 3.337953091 + halfpi, 7),
        BACK_RIGHT(11, 10, 0, 3.874, -0.392699241 + halfpi, 4);

        private final int driveID;
        private final int spinID;
        private final int encoderID;
        private final double absoluteEncoderOffset;
        private final double relativeEncoderOffset;
        private final int reedSwitchID;

        SwerveModuleConfig(int driveID, int spinID, int encoderID, double absoluteEncoderOffset, int reedSwitchID) {
            this(driveID, spinID, encoderID, absoluteEncoderOffset, 0, reedSwitchID);
        }

        SwerveModuleConfig(int driveID, int spinID, int encoderID, double absoluteEncoderOffset, double relativeEncoderOffset, int reedSwitchID) {
            this.driveID = driveID;
            this.spinID = spinID;
            this.encoderID = encoderID;
            this.absoluteEncoderOffset = absoluteEncoderOffset;
            this.reedSwitchID = reedSwitchID;
            this.relativeEncoderOffset = relativeEncoderOffset;
        }

        public int getDriveID() {
            return driveID;
        }

        public int getSpinID() {
            return spinID;
        }

        public int getEncoderID() {
            return encoderID;
        }

        @Deprecated
        public double getEncoderOffset() {
            return absoluteEncoderOffset;
        }

        public double getAbosoluteEncoderOffset() {
            return absoluteEncoderOffset;
        }

        public double getRelativeEncoderOffset() {
            return relativeEncoderOffset;
        }

        public int getReedSwitchID() {
            return reedSwitchID;
        }
    }

    /* Robot Width and Length Constants */
    public static final double motorLengthApartInches = 23.375;
    public static final double motorWidthApartInches = 23.0;

    /* Offset calculations */
    public static final double moduleXOffsetMeters = Units.inchesToMeters(motorLengthApartInches / 2);
    public static final double moduleYOffsetMeters = Units.inchesToMeters(motorWidthApartInches / 2);
/*
*              FRONT
*              (+x)
*               |
*               |
 * LEFT(-y)------------(+y)RIGHT
*               |
*               |
*              (-x)
*              BACK
 */
    /* Origin */
    public static final Translation2d origin = new Translation2d(0, 0);

    /* Motor offsets */
    public static final Translation2d frontRightOffsetMeters = new Translation2d(moduleXOffsetMeters,
            moduleYOffsetMeters);
    public static final Translation2d frontLeftOffsetMeters = new Translation2d(-moduleXOffsetMeters,
            moduleYOffsetMeters);
    public static final Translation2d backLeftOffsetMeters = new Translation2d(-moduleXOffsetMeters,
            -moduleYOffsetMeters);
    public static final Translation2d backRightOffsetMeters = new Translation2d(moduleXOffsetMeters,
            -moduleYOffsetMeters);

    /* Gear ratios */
    public static final double gearRatioSpin = 16.0;
    public static final double gearRatioDrive = 90.0/14.0; 
    public static final double wheelCircumference = 0.1016 * Math.PI;

    /* Motor maximum speed */
    public static final double driveMotorMaxRPM = 5676; 
    public static final double maxDriveSpeed = driveMotorMaxRPM / 60 / gearRatioDrive * wheelCircumference;

    public static final double driveMotorMaxSpeedRatio = 5.0; //unchecked (but very close)
    public static final double spinMotorMaxSpeedMetersPerSecond = 100.0;//unchecked (but also pretty close)
  
    public static final double anglePIDDefaultValue = 0.55;
    public static final double anglePIDDerivativeValue = 0.01;
    public static final double drivePIDDefaultValue = 0.6;

/* 
    ========
    BELT CONSTANTS 
    ========
*/
    public static final int beltMotorPort = 5;
    public static final int beltFanPort1 = 0;
    public static final int beltFanPort2 = 1;
    public static final double fanSpeed = 0.1;

/* 
    ========
    WHEELY ARM CONSTANTS 
    ========
*/
    public static final int wheelyArmUpperMotorPort = 6;
    public static final int wheelyArmLowerMotorPort = 5;
    
/* 
    ========
    SWITCHES 
    ========
*/
    public static final double reedSwitchDebounceTime = 0.1;
    public static final double zeroModulesDebounceTime = 0.5;
/* 
    ===========
    CONTROLLERS
    ===========
*/
    public static final int driveStickPort = 0;
    public static final int flightStickPort = 1;
    // The Driver Station joystick used for driving the robot.
    public static final CommandXboxController driverController = new CommandXboxController(Constants.driveStickPort);
    public static final Joystick driverJoystick = new Joystick(Constants.flightStickPort);
    public static final double chassisRotationsPerSecondMultiplier = Math.PI;

/* 
    =====
    AUTON 
    =====
*/  public static final double pastBias = 0.5;
    public static final double[] autonTargetXPose = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    public static final double[] autonTargetYPose = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    public static final double autonDistancePIDDefaultValue = 0.15;
    public static final double autonDistancePIDIntegralValue = 0;
    public static final double autonDistancePIDDerivativeValue = 0.005;

}
