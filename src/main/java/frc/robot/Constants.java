// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

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

    // Used for initializing swerve modules at startup
    public enum SwerveModuleConfig {
        FRONT_RIGHT(16, 17, 2, 0, 4),
        FRONT_LEFT(10, 11, 1, 0, 5),
        BACK_LEFT(12, 13, 0, 5.06, 6),
        BACK_RIGHT(15, 14, 3, 0, 7);

        private final int driveID;
        private final int spinID;
        private final int encoderID;
        private final double encoderOffset;
        private final int reedSwitchID;

        SwerveModuleConfig(int driveID, int spinID, int encoderID, double encoderOffset, int reedSwitchID) {
            this.driveID = driveID;
            this.spinID = spinID;
            this.encoderID = encoderID;
            this.encoderOffset = encoderOffset;
            this.reedSwitchID = reedSwitchID;
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

        public double getEncoderOffset() {
            return encoderOffset;
        }

        public int getReedSwitchID() {
            return reedSwitchID;
        }
    }

    public static final double reedSwitchDebounceTime = 0.1;
    // The port on the Driver Station that the joystick is connected to.
    public static final int driveStickPort = 0;
    public static final int flightStickPort = 1;
    // The Driver Station joystick used for driving the robot.
    public static final XboxController driverController = new XboxController(Constants.driveStickPort);
    public static final Joystick driverJoystick = new Joystick(Constants.flightStickPort);
    public static final double chassisRotationsPerSecondMultiplier = Math.PI;

    /* Gear ratio between the spin motor and wheel rotation */
    public static final double gearRatioSpin = 26.0 + 2.0 / 3.0;
    public static final double gearRatioDrive = 7.0 / 60; // what alex said, please don't kill me if it is wrong
    public static final double wheelCircumference = 0.1016 * Math.PI;
    public static final double driveMotorMaxRPM = 5676;
    public static final double maxDriveSpeed = driveMotorMaxRPM / 60 * gearRatioDrive * wheelCircumference;

    /* Motor maximum speed */
    public static final double driveMotorMaxSpeedRatio = 5.0;
    public static final double spinMotorMaxSpeedMetersPerSecond = 100.0;
    public static final double anglePIDDefaultValue = 0.4;
    public static final double anglePIDDerivativeValue = 0.01;
    public static final double drivePIDDefaultValue = 0.6;
    public static final double autonDistancePIDDefaultValue = 0.15;
    public static final double autonDistancePIDDerivativeValue = 0.005;
    public static final double autonDistancePIDIntegralValue = 0;

    /* Robot Width and Length Constants */
    public static final double motorLengthApartInches = 24.5625;
    public static final double motorWidthApartInches = 20.875;

    /* Offset calculations */
    public static final double moduleXOffsetMeters = Units.inchesToMeters(motorWidthApartInches / 2);
    public static final double moduleYOffsetMeters = Units.inchesToMeters(motorLengthApartInches / 2);

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

}
