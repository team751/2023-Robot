package frc.robot.commands;

import frc.robot.subsystems.drivetrain.SwerveDrive;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoLevel extends CommandBase{
    private AHRS navX2;
    private SwerveDrive swerveDrive;
    private PIDController levelXPIDController;
    private Debouncer debouncer;
    //private double ySpeed;    
    public AutoLevel(AHRS navX2, SwerveDrive swerveDrive){
        this.navX2 = navX2;
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        this.debouncer = new Debouncer(3);
        // should give a much smoother slowdown? I hope?
        levelXPIDController = new PIDController(0.2, 0, 0);
    }
    
    @Override 
    public void initialize(){
        SmartDashboard.putString("Current Mode","Auto-Level");
    }

    @Override
    public void execute(){
        // Robot wants to be at 0 degrees, PID controller to help
        double pitch = navX2.getPitch();
        double ySpeed = levelXPIDController.calculate(pitch, 0);
        // Caps the speed at 0.75 m/s
        if(Math.abs(ySpeed) > 0.75) ySpeed = 0.75 * Math.signum(ySpeed);
        //ySpeed = levelXPIDController.calculate(states[1], 0);
        //TODO: tune this value using the actual charge station
        if(Math.abs(pitch) < 4) ySpeed = 0;
        swerveDrive.drive(0, -ySpeed, 0,true);
        //finished = debouncer.calculate(states[0] < 3);
        SmartDashboard.putNumber("Speed For Level", ySpeed*-1);
    }

    @Override
    public boolean isFinished(){
        // Pitch in degrees
        return debouncer.calculate(Math.abs(navX2.getPitch()) < 10);
    }

    @Override
    public void end(boolean interrupted){
        swerveDrive.stop();
    }
}
