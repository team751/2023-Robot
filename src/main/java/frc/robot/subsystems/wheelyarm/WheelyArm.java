package frc.robot.subsystems.wheelyarm;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WheelyArm extends SubsystemBase {
    /** Creates a new WheelyArm. */
    private final CANSparkMax upperMotor;
    private final CANSparkMax lowerMotor;

    public WheelyArm(int upperMotorID, int lowerMotorID) {
        upperMotor = new CANSparkMax(upperMotorID, CANSparkMax.MotorType.kBrushless);
        lowerMotor = new CANSparkMax(lowerMotorID, CANSparkMax.MotorType.kBrushless);
    }
    
    public void setUpperMotor(double speed){
        upperMotor.set(speed);
    }

    private void setLowerMotor(double speed){
        lowerMotor.set(speed);
    }

    public void run(double speed){
        double upperMotorSpeed = speed * Constants.gearRatioWheelyArmUpper;
        double lowerMotorSpeed = speed * Constants.gearRatioWheelyArmLower;
        setUpperMotor(upperMotorSpeed);
        setLowerMotor(lowerMotorSpeed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
