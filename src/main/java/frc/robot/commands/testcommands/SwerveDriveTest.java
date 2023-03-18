package frc.robot.commands.testcommands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveModule;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Deprecated
public class SwerveDriveTest extends CommandBase{

    private final SwerveModule module;

    public SwerveDriveTest(SwerveModule backRightModule){
        this.module = backRightModule;
        SmartDashboard.putBoolean("Run Motor", false);
        SmartDashboard.putNumber("Motor Speed", 0.2);
        SmartDashboard.putNumber("Angle Set Point", 0);
        SmartDashboard.putBoolean("Reset Motor", false);
        SmartDashboard.putBoolean("Callibrate", false);
    }
    public void execute(){
        if(SmartDashboard.getBoolean("Run Motor", false)){
            module.setSpeed(SmartDashboard.getNumber("Motor Speed", 0.2));
        }else{
            module.setSpeed(0);
        }
        module.setAngle(SmartDashboard.getNumber("Angle Set Point", 0));
        
    }
    
}

