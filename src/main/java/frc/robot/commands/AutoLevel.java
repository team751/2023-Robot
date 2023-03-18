package frc.robot.commands;

import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyro.Odometry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoLevel extends CommandBase{
    private Odometry navX2;
    private SwerveDrive swerveDrive;
    private PIDController levelXPIDController;
    private double xSpeed;
    private Debouncer debouncer;
    //private double ySpeed;

    private boolean finished;
    
    public AutoLevel(Odometry navX2, SwerveDrive swerveDrive){
        this.navX2 = navX2;
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        this.debouncer = new Debouncer(0.5);
        this.finished = false;
        levelXPIDController = new PIDController(0.1, 0.1, 0.2);
        xSpeed = 0;
        //ySpeed = 0;
    }
    
    @Override 
    public void initialize(){
        SmartDashboard.putString("Current Mode","Auto-Level");
    }

    @Override
    public void execute(){
        xSpeed = levelXPIDController.calculate(navX2.getXAngle(), 0);
        //ySpeed = levelXPIDController.calculate(states[1], 0);
        swerveDrive.drive(xSpeed, 0, 0,false);
        //finished = debouncer.calculate(states[0] < 3);
    }

    @Override
    public boolean isFinished(){
        return finished;
    }

    @Override
    public void end(boolean interrupted){
    }
}
