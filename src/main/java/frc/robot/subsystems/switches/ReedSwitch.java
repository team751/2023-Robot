package frc.robot.subsystems.switches;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ReedSwitch extends SubsystemBase {
    private DigitalInput reedSwitch;

    public ReedSwitch(int portID) {
        reedSwitch = new DigitalInput(portID);
    }

    public ReedSwitch(int portID, String name) {
        this(portID);
        setName(name);
    }

    public boolean get() {
        // negate since default is true
        return reedSwitch.get();
    }

    public void debugPutValues() {
        SmartDashboard.putBoolean(this.getName() + " Reed Switch", this.get());
    }
}
