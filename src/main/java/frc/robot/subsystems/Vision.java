// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  
  private String name;
  public Vision(String name, double[] config) {
    this.name = name;
    LimelightHelpers.setCameraPose_RobotSpace(name, config[0], config[1], config[2], config[3], config[4], config[5]);

    // need to run this before using MegaTag2, according to docs
    // yaw:0 is facing red alliance wall, the rest say "unnecessary" so ¯\_(ツ)_/¯
    LimelightHelpers.SetRobotOrientation(name, 0, 0, 0, 0, 0, 0);
  }

  public double aimWithVision(){
    double kP = 0.015;
    double targetingAngularVelocity = LimelightHelpers.getTX(name) * kP;
    targetingAngularVelocity *= SwerveConstants.maxTurnSpeed;
    targetingAngularVelocity *= -1;
    return targetingAngularVelocity;
  }

  public double getDistToRPMVal(){
    return VisionConstants.distanceToRPMRatio;
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
    return (distance * VisionConstants.distanceToRPMRatio) + VisionConstants.baseRPM;

  }

  public boolean hasValidIDs(){
    return LimelightHelpers.getTV(name);
  }

  public Pose2d getPose(){
    return LimelightHelpers.getBotPose2d(name);
  }


  /**
   * 
   * @return
   */
  public LimelightHelpers.PoseEstimate getMegaTag2(Pose2d pose){
    LimelightHelpers.SetRobotOrientation(name, pose.getRotation().getRadians(), 0, 0, 0, 0, 0);
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
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

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);

    builder.addBooleanProperty("Has AprilTag", this::hasValidIDs, null);

    builder.addDoubleProperty("MegaTag2/X", 
      ()->LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).pose.getX(), null);
    builder.addDoubleProperty("MegaTag2/Y", 
      ()->LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).pose.getY(), null);
    builder.addDoubleProperty("MegaTag2/Rotation (rad)", 
      ()->LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).pose.getRotation().getRadians(), null);

    builder.addDoubleProperty("Pose/X", ()->LimelightHelpers.getBotPose2d(name).getX(), null);
    builder.addDoubleProperty("Pose/Y", ()->LimelightHelpers.getBotPose2d(name).getY(), null);
    builder.addDoubleProperty("Pose/Rotation (rad)", 
      ()->LimelightHelpers.getBotPose2d(name).getRotation().getRadians(), null);
  }
}
