package frc.robot.subsystems.gyro;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class NavX2CompFilter {
    private Odometry navX2;
    private double[] filteredAngles;
    private double deltaTime;
    private double olderTime;
    private double percentage;

    public NavX2CompFilter(){
        navX2 = Odometry.getInstance();
        filteredAngles = new double[3];
        deltaTime = 0.05;
        olderTime = System.currentTimeMillis();
    }

    public void getAngle() {
        deltaTime = (System.currentTimeMillis() - olderTime) / 1000;
        olderTime = System.currentTimeMillis();
    
        if (deltaTime > 0.1) {
            percentage = 1;
        } else {
            percentage = 0.2;
        }

        
        double angleX = percentage*getXAccelAngle()+(1-percentage)*(filteredAngles[0]+Math.PI/180*navX2.getRawGyroSpeedZ()*deltaTime);
        double angleY = percentage*getYAccelAngle()+(1-percentage)*(filteredAngles[1]+Math.PI/180*navX2.getRawGyroSpeedY()*deltaTime);
        filteredAngles[0] = angleX;
        filteredAngles[1] = angleY;
    }
    //none filtered angle based on accel 
    public double getXAccelAngle(){
        double result=Math.atan2(navX2.getRawZAccel(), Math.sqrt(Math.pow(navX2.getRawXAccel(), 2) + Math.pow(navX2.getRawYAccel(), 2)));
        return result;
    }
    public double getYAccelAngle(){
        double result=Math.atan2(navX2.getRawYAccel(), Math.sqrt(Math.pow(navX2.getRawZAccel(), 2) + Math.pow(navX2.getRawXAccel(), 2)));
        return result;
  }
    public double getFilteredXAngle(){
        getAngle();
        return filteredAngles[0];
    }
    public double getFilteredYAngle(){
        getAngle();
        return filteredAngles[1];
    }

    // past velocity and past acceleration can not be calculated with the sensor data as that removes the filtering so it must be identified from the previous iteration
  private double linearInterpolation(double pastDisplacement, double pastVelocity, double pastAcc, double sensorDisplacement) {
    double predictedDisplacement = pastDisplacement + pastVelocity * deltaTime+ 0.5 * pastAcc * deltaTime * deltaTime;
    return predictedDisplacement*(1-Constants.pastBias)+Constants.pastBias*sensorDisplacement;
  }

  private double[] updateInterpolation(double[] initialStates, double sensorData){
    double[] finalStates = new double[3];
    finalStates[2] = linearInterpolation(initialStates[2], initialStates[1], initialStates[0], sensorData);
    finalStates[1] = (finalStates[2]-initialStates[2])/deltaTime;
    finalStates[0] = (finalStates[1]-initialStates[1])/deltaTime;
    return finalStates;
  }
  public void debugPrintOut(){
    SmartDashboard.putNumber("Please work X", getFilteredXAngle());
    SmartDashboard.putNumber("Please work Y", getFilteredYAngle());
  }

}