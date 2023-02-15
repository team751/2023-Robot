package frc.robot.subsystems.gyro;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;

// Uses multiple inputs from the gyroscope to make the values more accurate
public class ComplementaryFilter extends SubsystemBase {
    private Gyro sensor;
    private double[] anglePrevious;
    private double deltaTime;
    private double olderTime;
    private double percentage = 0.2;
    private double[] angleError = { 0.0, 0.0, 0.0 };
    private ArrayList<Double> pastAverage = new ArrayList<Double>();

    public ComplementaryFilter() {
        sensor = new Gyro();
        sensor.calibrate();
        anglePrevious = new double[3];
        deltaTime = 0.05;
        olderTime = System.currentTimeMillis();
    }

    public double[] getAngle() {
        deltaTime = (System.currentTimeMillis() - olderTime) / 1000;
        olderTime = System.currentTimeMillis();
        // System.out.println(deltaTime);
        if (deltaTime > 1) {
            deltaTime = 0;
            percentage = 1;
            sensor.calibrate();
        } else {
            percentage = 0.2;
        }
        double[] angle = new double[3];
        double[] gyro = sensor.getRotationalVelocity();
        double[] accel = sensor.getAccelerationAngle();
        angle[0] = (1 - percentage) * (anglePrevious[0] + gyro[0] * deltaTime) + percentage * accel[0];
        angle[1] = (1 - percentage) * (anglePrevious[1] + gyro[1] * deltaTime) + percentage * accel[1];
        angle[2] = (1 - percentage) * (anglePrevious[2] + gyro[2] * deltaTime) + percentage * accel[2];
        anglePrevious[0] = angle[0];
        anglePrevious[1] = angle[1];
        anglePrevious[2] = angle[2];
        angle[0] = Math.toRadians(angle[0]) - angleError[0];
        angle[1] = Math.toRadians(angle[1]) - angleError[1];
        angle[2] = Math.toRadians(angle[2]) - angleError[2];

        // rolling average filter(stupid)
        pastAverage.add(angle[0]);
        if (pastAverage.size() > 1) {
            pastAverage.remove(0);
        }
        double sum = 0;
        for (double d : pastAverage) {
            sum += d;
        }
        angle[0] = sum / pastAverage.size();
        return angle;
    }

    public void calibrate() {
        sensor.calibrate();
        double[] angleError = getAngle();
        this.angleError = angleError;
    }

    public void debugAngle() {
        double[] angle = getAngle();
        SmartDashboard.putNumber("filteredX", Math.toDegrees(angle[0]));
        SmartDashboard.putNumber("FilteredY", Math.toDegrees(angle[1]));
    }

}