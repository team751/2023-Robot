package frc.robot.subsystems.switches;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ReedSwitch extends SubsystemBase {
    private DigitalInput reedSwitch;
    private Debouncer debouncer;

    public ReedSwitch(int portID) {
        reedSwitch = new DigitalInput(portID);
        debouncer = new Debouncer(Constants.reedSwitchDebounceTime, Debouncer.DebounceType.kBoth);
    }

    public ReedSwitch(int portID, String name) {
        this(portID);
        setName(name);
    }

    public boolean get() {
        // negate since default is true
        return !debouncer.calculate(reedSwitch.get());
    }

    public void debugPutValues() {
        SmartDashboard.putBoolean(this.getName() + " Reed Switch", this.get());
    }
}
