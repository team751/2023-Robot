package frc.robot.subsystems.camera;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private final NetworkTable table;

    private final NetworkTableEntry target;
    private final NetworkTableEntry botPos;
    private final NetworkTableEntry camPosTagSpace;
    private final NetworkTableEntry tagPosBotSpace;

    public Limelight() { 
        table = NetworkTableInstance.getDefault().getTable("limelight");
        target = table.getEntry("tv");
        botPos = table.getEntry("botpose");
        camPosTagSpace = table.getEntry("campose");
        tagPosBotSpace = table.getEntry("targetpose");

    }

    public double[] getValues() {
        double[] position = null;
        if (target.getDouble(0) > 0.9) {
            // position = tagPos.getDoubleArray(new double[6]);
            position = tagPosBotSpace.getDoubleArray(new double[6]);
        }
        return position;
    }

    public void debugDisplayValues() {
        double[] newValues = getValues();
        if (newValues == null || newValues.length < 3)
            return;
        SmartDashboard.putNumber("Robot X Position", newValues[0]);
        SmartDashboard.putNumber("Robot Y Position", newValues[1]);
        SmartDashboard.putNumber("Robot Z Position", newValues[2]);
        SmartDashboard.putNumber("Robot X Rotation", newValues[3]);
        SmartDashboard.putNumber("Robot Y Rotation", newValues[4]);
        SmartDashboard.putNumber("Robot Z Rotation", newValues[5]);
    }
}
