package frc.robot.subsystems.absencoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AbsoluteEncoder extends SubsystemBase {

    // PWM port
    private final DutyCycleEncoder absEncoder;

    public AbsoluteEncoder(int PWMport) {
        absEncoder = new DutyCycleEncoder(PWMport);
        absEncoder.setDutyCycleRange(0, 1024);
        absEncoder.reset();
    }

    public void debugDisplayValues() {
        debugDisplayValues("Absolute Encoder");
    }

    public void debugDisplayValues(String moduleName) {
        SmartDashboard.putNumber(moduleName + "Angle (Degrees)", absEncoder.getAbsolutePosition() * 360);
    }

    public double getPositionRadians() {
        return absEncoder.getAbsolutePosition() * 2 * Math.PI;
    }

}
