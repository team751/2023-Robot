package frc.robot.commands;

import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.gyro.NavX2CompFilter;
import frc.robot.subsystems.gyro.Odometry;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoLevel extends CommandBase{
    private AHRS navX2;
    private SwerveDrive swerveDrive;
    private PIDController levelXPIDController;
    private SlewRateLimiter limiter;
    private Debouncer debouncer;
    //private double ySpeed;

    private boolean finished;
    
    public AutoLevel(AHRS navX2, SwerveDrive swerveDrive){
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
        // Robot wants to be at 0 degrees, PID controller to help
        double ySpeed = levelXPIDController.calculate(navX2.getPitch(), 0);
        // Caps the speed at 0.75 m/s
        if(Math.abs(ySpeed) > 0.75) ySpeed = 0.75 * Math.signum(ySpeed);
        //ySpeed = levelXPIDController.calculate(states[1], 0);
        swerveDrive.drive(0, ySpeed, 0,true);
        //finished = debouncer.calculate(states[0] < 3);
        SmartDashboard.putNumber("Speed For Level", ySpeed*-1);
    }

    @Override
    public boolean isFinished(){
        return debouncer.calculate(navX2.getPitch() < 6);
    }

    @Override
    public void end(boolean interrupted){
        swerveDrive.stop();
    }
}
