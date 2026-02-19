// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private double distToRPM = 750;
  private String name;
  public Vision(String name, double[] config) {
    this.name = name;
    LimelightHelpers.setCameraPose_RobotSpace(name, config[0], config[1], config[2], config[3], config[4], config[5]);
    
  }

  public double aimWithVision(){
    double kP = 0.03;
    double targetingAngularVelocity = LimelightHelpers.getTX(name) * kP;
    targetingAngularVelocity *= SwerveConstants.maxTurnSpeed;
    targetingAngularVelocity *= -1;
    return targetingAngularVelocity;
  }

  public double getDistToRPMVal(){
    return distToRPM;
  }

  public void setDistToRPMVal(double rpmScaler){
    distToRPM = rpmScaler;
  }

  public double rangeWithVision(double range){
    double  kP = 0.1;
    double targetingRangeVelocity = (LimelightHelpers.getTY(name) * kP) + range;
    targetingRangeVelocity *= SwerveConstants.maxDriveSpeed;
    return targetingRangeVelocity;
  }

  public void setValidIDs(int[] validIDs){
    LimelightHelpers.SetFiducialIDFiltersOverride(name, validIDs);
  }


  //TODO: find actual constant and verify range works as intended
  /**
   * finds a target RPM based on the distance to a target
   * @return a number of rpm based on distance from the target
   */
  public double calculateRPMFromRange(){
    double camera_height = VisionConstants.aimingConfig[2];
    double tag_height = VisionConstants.aimingTagHeight;
    double angle = Math.toRadians(LimelightHelpers.getTY(name) + VisionConstants.aimingConfig[4]);
    double distance = (tag_height - camera_height) / Math.tan(angle);
    return distance * distToRPM;

  }

  public boolean hasValidIDs(){
    return LimelightHelpers.getTV(name);
  }



  public LimelightHelpers.PoseEstimate getMegaTag2(){
    LimelightHelpers.PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    return megaTag2;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(DriverStation.isDisabled() || DriverStation.isEStopped()){
      LimelightHelpers.SetThrottle(name, 200);    //reduce processing when robot isn't actually running
    }else{
      LimelightHelpers.SetThrottle(name, 0);
    }
  }
}
