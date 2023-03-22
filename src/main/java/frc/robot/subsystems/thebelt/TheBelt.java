package frc.robot.subsystems.thebelt;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TheBelt extends SubsystemBase {

    private final CANSparkMax motor;
    private final PWM fan1;
    private final PWM fan2;

    /** Creates a new ExampleSubsystem. */
    public TheBelt(int motorID, int PWMFan1, int PWMFan2) {
        motor = new CANSparkMax(motorID, CANSparkMax.MotorType.kBrushless);
        fan1 = new PWM(PWMFan1);
        fan2 = new PWM(PWMFan2);
    }

    public void run(double speed){
        motor.set(speed);
        fan1.setSpeed(Constants.fanSpeed);
        fan2.setSpeed(Constants.fanSpeed);
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

