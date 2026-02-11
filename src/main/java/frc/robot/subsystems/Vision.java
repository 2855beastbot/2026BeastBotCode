// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private String name;
  public Vision(String name, double[] config) {
    this.name = name;
    LimelightHelpers.setCameraPose_RobotSpace(name, config[0], config[1], config[2], config[3], config[4], config[5]);
    
  }

  public double aimWithVision(){
    double kP = 0.017;
    double targetingAngularVelocity = LimelightHelpers.getTX(name) * kP;
    targetingAngularVelocity *= SwerveConstants.maxTurnSpeed;
    return targetingAngularVelocity;
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
