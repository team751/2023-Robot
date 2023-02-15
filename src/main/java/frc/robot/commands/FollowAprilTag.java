package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.camera.Limelight;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.gyro.ComplementaryFilter;
import frc.robot.subsystems.gyro.NavX2;

public class FollowAprilTag extends CommandBase {
    private final SwerveDrive swerb;
    private final NavX2 navX2;
    private ComplementaryFilter filteredAngles;
    private final Limelight limelight;
    private final double xTarget;
    private final double yTarget;
    private double robotXPos;
    private double robotYPos;
    private double vx;
    private double vy;
    private final PIDController distanceController;
    private double deltaTime;
    private double olderTime;
    private double[] gyroStates;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public FollowAprilTag(SwerveDrive subsystem, Limelight limelight, ComplementaryFilter filteredAngles, NavX2 navX2) {
        this.limelight = limelight;
        this.navX2 = navX2;
        swerb = subsystem;

        this.filteredAngles = filteredAngles;
        addRequirements(subsystem);
        xTarget = 1;
        yTarget = 0;
        robotXPos = xTarget;
        robotYPos = yTarget;
        deltaTime = 0;
        olderTime = 0;
        gyroStates = new double[7];
        distanceController = new PIDController(Constants.autonDistancePIDDefaultValue,
                Constants.autonDistancePIDIntegralValue, Constants.autonDistancePIDDerivativeValue);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        deltaTime = (System.currentTimeMillis() - olderTime) / 1000;
        olderTime = System.currentTimeMillis();
        gyroStates = navX2.getValues();
        navX2.debugPutValues();
        limelight.debugDisplayValues();
        double[] values = limelight.getValues();
        if (values != null && values.length >= 3) {
            robotXPos = values[2];
            robotYPos = values[1];
        } else {
            robotXPos = robotXPos + vx * deltaTime;
            robotYPos = robotYPos + vy * deltaTime;
        }
        vx = distanceController.calculate(robotXPos, xTarget) * Constants.maxDriveSpeed / 2;
        vy = distanceController.calculate(robotYPos, yTarget) * Constants.maxDriveSpeed / 2;
        

        filteredAngles.debugAngle();
        swerb.drive(vx, vy, 0);

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
