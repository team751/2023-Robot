package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.camera.Limelight;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.ComplementaryFilter;
import frc.robot.subsystems.gyro.NavX2;

public class SwerveDriveCommand extends CommandBase {
    // Swerb controller
    private final SwerveDrive swerveSubsystem;
    private final Limelight limelight;
    // Joystick Limiters
    private final SlewRateLimiter vxFLimiter;
    private final SlewRateLimiter vyFLimiter;
    // Gyroscope things
    private final ComplementaryFilter filteredAngles;
    private final Gyro rawGyro;
    private NavX2 navX2;
    // Camera
    // private final Limelight limelight;
    // Auto Level
    private final PIDController levelPIDController;
    // Boolean values for mode toggles
    private boolean autoLevel;
    private boolean zeroModules;
    private double height;
    private double olderTime;
    private double deltaTime;
    private double netDistance;
    private double netSpeed;

    private boolean bIsPressed;
    private boolean aIsPressed;

    /**
     * Creates a new SwerveDriveCommand.
     * This is the main command class for the robot during Teleop.
     * It handles all the swerve drive and limelight functionality,
     * as well as the auto-leveling with the press of a button
     *
     * @param subsystem The subsystem used by this command.
     */
    public SwerveDriveCommand(SwerveDrive subsystem, Limelight limelight, ComplementaryFilter filteredAngles, NavX2 navX2) {
        this.limelight = limelight;
        rawGyro = new Gyro();
        this.navX2 = navX2;
        netDistance = 0;
        netSpeed = 0;
        olderTime = 0;
        deltaTime = 0;
        zeroModules = false;
        bIsPressed = false;
        aIsPressed = false;
        autoLevel = false;
        swerveSubsystem = subsystem;
        addRequirements(subsystem);
        height = 0;
        // Gyroscope
        this.filteredAngles = filteredAngles;
        autoLevel = false;
        // Camera
        SmartDashboard.putNumber("Left Stick Angle (Radians)", 0);
        SmartDashboard.putNumber("Left Stick Magnitude", 0);
        SmartDashboard.putNumber("Right Stick Rotation", 0);

        vxFLimiter = new SlewRateLimiter(4);
        vyFLimiter = new SlewRateLimiter(4);
        levelPIDController = new PIDController(0.5, 0, 0);
        SmartDashboard.putBoolean("Xbox Controller", true);
        SmartDashboard.putBoolean("Callibrate", false);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get Gyro Readings
        filteredAngles.debugAngle();

        // Get camera values
        limelight.debugDisplayValues();
        navX2.debugPutValues();
        if(SmartDashboard.getBoolean("Callibrate", false)){
            navX2.callibrate();
        }

        // Get Joystick Values vx and vy are in meters per second
        double vxStick;
        double vyStick;
        double rpsStick;
        if (SmartDashboard.getBoolean("Xbox Controller", true)) {
            vxStick = Constants.driverController.getLeftX();
            vyStick = Constants.driverController.getLeftY();
            rpsStick = Constants.driverController.getRightX();
        } else {
            vxStick = Constants.driverJoystick.getX();
            vyStick = Constants.driverJoystick.getY();
            rpsStick = Constants.driverJoystick.getZ();
        }

        double vx = vxFLimiter.calculate(vxStick) * Constants.maxDriveSpeed;
        double vy = vyFLimiter.calculate(vyStick) * Constants.maxDriveSpeed;
        double rotationsPerSecond = rpsStick * Constants.chassisRotationsPerSecondMultiplier;

        // Speed deadband
        if (Math.sqrt(vx * vx + vy * vy) < 0.2) {
            vx = 0;
            vy = 0;
        }

        // TODO: weird behavior where this only works when "A" is held down
        // TODO: perhaps have the function return true if completed?
        if (Constants.driverController.getBButtonPressed() != bIsPressed) {
            autoLevel = !autoLevel;
            olderTime = 0;
            netSpeed = 0;
            netDistance = 0;
            SmartDashboard.putString("Current Mode", "Auto Level");
        }
        bIsPressed = Constants.driverController.getBButtonPressed();

        if (Constants.driverController.getAButtonPressed()) {
            zeroModules = !zeroModules;
            SmartDashboard.putString("Current Mode", "Zeroing Modules");
        }

        if (Constants.driverController.getStartButtonPressed()) {
            rawGyro.calibrate();
            filteredAngles.calibrate();
        }

        if (zeroModules) {
            swerveSubsystem.zeroModules();
        } else if (autoLevel) {
            autoLevel();
        } else {
            swerveSubsystem.drive(vx, vy, rotationsPerSecond);
            SmartDashboard.putString("Current Mode", "Drive");
        }

        // Smart dashboard controller
        SmartDashboard.putNumber("Left Stick Angle (Radians)", Math.atan2(vx, vy));
        SmartDashboard.putNumber("Left Stick Magnitude", Math.sqrt(vx * vx + vy * vy));
        SmartDashboard.putNumber("Right Stick Rotation", rotationsPerSecond);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // swerveSubsystem.stop();
    }

    public double calculateZAccel() {
        double[] angles = filteredAngles.getAngle();
        double[] accels = rawGyro.getAcceleration();
        double zAccelM = accels[2] * Math.cos(Math.abs(angles[1])) * Math.cos(Math.abs(angles[0]))
                + accels[0] * Math.sin(Math.abs(angles[0]))
                + accels[1] * Math.sin(Math.abs(angles[1]));
        float zAccelSign = 0;
        return zAccelM;
    }

    public void autoLevel() {
        if (olderTime == 0) {
            olderTime = System.currentTimeMillis();
            return;
        }
        deltaTime = (System.currentTimeMillis() - olderTime) / 1000;
        olderTime = System.currentTimeMillis();
        double[] accel = rawGyro.getAcceleration();
        double[] angles = filteredAngles.getAngle();
        double netZAccel = accel[0] * Math.sin(angles[0])
                + accel[1] * Math.sin(angles[1])
                + accel[2] * Math.sin(Math.PI / 2 - angles[0]) * Math.sin(Math.PI / 2 - angles[1]);
        // m/s^2
        double netAccel1 = calculateZAccel();
        SmartDashboard.putNumber("haha", netAccel1);
        netSpeed += netAccel1 * deltaTime;

        netDistance += netSpeed * deltaTime;
        System.out.println(netDistance);
        // TODO: fix this
        // swerveSubsystem.drive(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
