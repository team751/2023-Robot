package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;


//TODO: account for new position on the robot
public class Odometry {
    private static Odometry instance;

    private AHRS navX2;

    private Odometry() {
        navX2 = new AHRS(SPI.Port.kMXP);
        navX2.reset();
        navX2.zeroYaw();
        navX2.resetDisplacement();
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                navX2.calibrate();
                navX2.zeroYaw();
                navX2.resetDisplacement();
            }catch(Exception e){}
        }).start();
    }

    public static Odometry getInstance(){
        if(instance == null){
            instance = new Odometry();
        }
        return instance;
    }


    public double getRawXAccel(){
        return navX2.getRawAccelX();
    }

    public double getRawYAccel(){
        return navX2.getRawAccelY();
    }

    public double getRawZAccel(){
        return navX2.getRawAccelZ();
    }

    public double getRawGyroSpeedX(){
        return navX2.getRawGyroX();
    }
    public double getRawGyroSpeedY(){
        return navX2.getRawGyroY();
    }
    public double getRawGyroSpeedZ(){
        return navX2.getRawGyroZ();
    }

    @Deprecated
    public double getDisplacementZ() {
        return navX2.getDisplacementZ();
    }
    @Deprecated
    public double getDisplacementY() {
        return navX2.getDisplacementY();
    }
    @Deprecated
    public double getDisplacementX() {
        return navX2.getDisplacementX();
    }
    @Deprecated
    public Rotation2d getHeading() {
        return new Rotation2d(Units.degreesToRadians(navX2.getAngle()));
    }
    @Deprecated
    public double getZAngle() {
        return navX2.getYaw();
    }
    @Deprecated
    public double getYAngle() {
        return navX2.getPitch();
    }
    @Deprecated
    public double getXAngle() {
        return navX2.getRoll();
    }





    public void debugPutValues() {
        SmartDashboard.putNumber("NavX2 X Angle (Degrees)", getXAngle());
        SmartDashboard.putNumber("NavX2 Y Angle (Degrees)", getYAngle());
        SmartDashboard.putNumber("NavX2 Z Angle (Degrees)", getZAngle());
        SmartDashboard.putNumber("NavX2 Heading", getHeading().getDegrees());
        SmartDashboard.putNumber("NavX2 Z Displacement", getDisplacementZ());
        SmartDashboard.putNumber("NavX2 Y Displacement", getDisplacementY());
        SmartDashboard.putNumber("NavX2 X Displacement", getDisplacementX());
    }
}