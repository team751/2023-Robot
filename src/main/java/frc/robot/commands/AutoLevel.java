package frc.robot.commands;

import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyro.NavX2CompFilter;
import frc.robot.subsystems.gyro.Odometry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoLevel extends CommandBase{
    private Odometry navX2;
    private SwerveDrive swerveDrive;
    private PIDController levelXPIDController;
    private SlewRateLimiter limiter;
    private Debouncer debouncer;
    //private double ySpeed;

    private boolean finished;
    
    public AutoLevel(Odometry navX2, SwerveDrive swerveDrive){
        this.navX2 = navX2;
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        this.debouncer = new Debouncer(3);
        this.finished = false;
        levelXPIDController = new PIDController(0.6, 0, 0);
        limiter = new SlewRateLimiter(1);
        //ySpeed = 0;
    }
    
    @Override 
    public void initialize(){
        SmartDashboard.putString("Current Mode","Auto-Level");
    }

    @Override
    public void execute(){
        double ySpeed = levelXPIDController.calculate(navX2.getYAngle()+88, 0);
        if(Math.abs(ySpeed) > 1) ySpeed = 1 * Math.signum(ySpeed);
        //ySpeed = levelXPIDController.calculate(states[1], 0);
        swerveDrive.drive(0, ySpeed, 0,false);
        //finished = debouncer.calculate(states[0] < 3);
        SmartDashboard.putNumber("Speed For Level", ySpeed*-1);
        if(debouncer.calculate(navX2.getYAngle()+88 < 6)) this.end(false);
    }

    @Override
    public boolean isFinished(){
        return finished;
    }

    @Override
    public void end(boolean interrupted){
        swerveDrive.stop();
    }
}
