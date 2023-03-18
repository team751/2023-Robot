package frc.robot.subsystems.thebelt;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TheBelt extends SubsystemBase {

    private final CANSparkMax motor;

    /** Creates a new ExampleSubsystem. */
    public TheBelt(int motorID) {
        motor = new CANSparkMax(motorID, CANSparkMax.MotorType.kBrushless);
    }

    public void run(double speed){
        motor.set(speed);
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

