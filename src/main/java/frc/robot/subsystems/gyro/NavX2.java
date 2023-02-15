package frc.robot.subsystems.gyro;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.kauailabs.navx.frc.AHRS;

public class NavX2 {
    private AHRS navX2;
    private double zDisplacement;
    private double oldZDisplacement;
    private double zDisplacementoffset;
    private double zCalibrateCount;
    private double zCalibrateTime;
    private ScheduledExecutorService calibrationThread;
    private Runnable callibrator;
    private double initialCalibrateTime;
    private double[] gyroStates;


    public NavX2() {
        initialCalibrateTime = System.currentTimeMillis()/1000;
        oldZDisplacement = 0;
        gyroStates = new double[7];
        zDisplacementoffset = 0;
        calibrationThread = Executors.newScheduledThreadPool(1);
        zCalibrateTime = 100;
        navX2 = new AHRS(SPI.Port.kMXP);
        navX2.calibrate();
        navX2.zeroYaw();
        zCalibrateCount = 0;
        callibrator = new Runnable() { 
            @Override
            public void run() {

                zDisplacementoffset = (zDisplacementoffset+navX2.getDisplacementZ()-oldZDisplacement/(zCalibrateTime/1000))/2;
                if (zCalibrateCount>10){
                    calibrationThread.shutdown();
                }
            }
            
        };
        callibrate();
    }

    public void callibrate() {
        initialCalibrateTime = System.currentTimeMillis()/1000;
        calibrationThread.scheduleAtFixedRate(callibrator, 0, Math.round(zCalibrateTime), TimeUnit.MILLISECONDS);
    }
    public double getCalibratedDisplacementZ() {
        zDisplacement = navX2.getDisplacementZ()-((System.currentTimeMillis()/1000-initialCalibrateTime)*zDisplacementoffset);
        return zDisplacement;
    }

    public void dumbCallibration(){
        navX2.calibrate();
    }
    public double[] getValues(){
        double[] values = new double[7];
        values[0] = navX2.getRoll();
        values[1] = navX2.getPitch();
        values[2] = navX2.getYaw();
        values[3] = navX2.getFusedHeading();
        values[4] = getCalibratedDisplacementZ();
        values[5] = navX2.getDisplacementY();
        values[6] = navX2.getDisplacementX();
        return values;
    }
    public void debugPutValues() {
        gyroStates = getValues();
        SmartDashboard.putNumber("NavX2 X Angles", gyroStates[0]);
        SmartDashboard.putNumber("NavX2 Y Angles", gyroStates[1]);
        SmartDashboard.putNumber("NavX2 Z Angles", gyroStates[2]);
        SmartDashboard.putNumber("NavX2 Heading", gyroStates[3]);
        SmartDashboard.putNumber("NavX2 Z Displacement", gyroStates[4]);
        SmartDashboard.putNumber("NavX2 Y Displacement", gyroStates[5]);
        SmartDashboard.putNumber("NavX2 X Displacement", gyroStates[6]);
    }
}