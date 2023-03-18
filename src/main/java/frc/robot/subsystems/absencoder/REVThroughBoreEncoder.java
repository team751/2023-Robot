package frc.robot.subsystems.absencoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class REVThroughBoreEncoder extends SubsystemBase {

    // PWM port
    private final DutyCycleEncoder absEncoder;

    public REVThroughBoreEncoder(int DIOport) {
        absEncoder = new DutyCycleEncoder(DIOport);
        //Through-bore encoder in duty cycle mode = 1024 U/rev
        absEncoder.setDutyCycleRange(0, 1024);
        absEncoder.reset();
    }

    public REVThroughBoreEncoder(int DIOport, String moduleName){
        this(DIOport);
        this.setName(moduleName);
    }

    public void debugDisplayValues() {
        SmartDashboard.putNumber(this.getName() + "Angle (Degrees)", absEncoder.getAbsolutePosition() * 360);
    }

    public double getAbsolutePositionRadians() {
        return absEncoder.getAbsolutePosition() * 2 * Math.PI;
    }

}
