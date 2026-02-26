// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */
  private SwerveDrive swerveDrive;
  private RobotConfig config;
  private Vision aimingCamera = new Vision(VisionConstants.aimingLimelightName, VisionConstants.aimingConfig);
  private final PIDController pointToPosePID = new PIDController(2.0, 0.0, 0.1);
  private Pose2d targetHub;

  public Swerve() {
    double maximumSpeed = Units.feetToMeters(4.5);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    try{
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }catch(IOException e){
      throw new RuntimeException(e);
    }

    try{
      config = RobotConfig.fromGUISettings();
    }catch(Exception e){
      e.printStackTrace();
    }
    configureAutoBuilder();
    swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(9999999, 9999999, 9999999)); // higher number means less trust
    //reiously0.7,0.7,9999999
    pointToPosePID.enableContinuousInput(-Math.PI, Math.PI);
    pointToPosePID.setTolerance(2.0);

    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
        targetHub = (alliance.get() == Alliance.Blue) ? VisionConstants.blueHub : VisionConstants.redHub;
      }else{
        targetHub = VisionConstants.blueHub;
      }
  }

  public double getMaxDriveSpeed(){
    return SwerveConstants.maxDriveSpeed;
  }

  public double getMaxTurnSpeed(){
    return SwerveConstants.maxTurnSpeed;
  }

  /**
   * Primary method for driving robot
   * @param translation meters per second
   * @param rotation radians per second
   * @param fieldRelative
   * @param isOpenLoop
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
    swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
  }

  /**
   * Drive robot while pointing at alliance hub
   * @param translation x and y speeds to drive at
   */
  public void drivePose(Translation2d translation){
    swerveDrive.drive(
      swerveDrive.getSwerveController().getTargetSpeeds(
        translation.getX(), 
        translation.getY(), 
        getPointAtPoseAngle(targetHub).getRadians(),
        getPose2d().getRotation().getRadians(),
        //pointToPosePID.calculate(getPose2d().getRotation().getRadians(), getPointAtPoseAngle(targetHub).getRadians()),
        getMaxTurnSpeed())
    );
  }

  public void setXMode(){
    swerveDrive.lockPose();
  }

  public Pose2d getPose2d(){
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d pose){
    swerveDrive.resetOdometry(pose);
  }


  /**
   * updates the target hub based on driver station
   */
  public void updateAlliance(){
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
        targetHub = (alliance.get() == Alliance.Blue) ? VisionConstants.blueHub : VisionConstants.redHub;
      }else{
        targetHub = VisionConstants.blueHub;
      }
  }


  /**
   * resets the pose of the robot to the passed in pose, rotating by 180  if in red alliance
   * @param pose the pose to set to
   */
  public void resetOdometryWithAlliance(Pose2d pose){
    if(targetHub == VisionConstants.blueHub){
      swerveDrive.resetOdometry(pose);
    }else{
      swerveDrive.resetOdometry(new Pose2d(pose.getTranslation(), new Rotation2d(pose.getRotation().getRadians() + Math.PI)));
    }
  }

  public ChassisSpeeds getChassisSpeeds(){
    return swerveDrive.getRobotVelocity();
  }

  public void setRobotRelativeSpeeds(ChassisSpeeds speed){
    swerveDrive.setChassisSpeeds(speed);
  }

  /**
   * Returns the angle of a line pointing from the robot's position to a specific position on the field
   * @param targetPose position to point at
   * @return angle from robot to target position
   */
  public Rotation2d getPointAtPoseAngle(Pose2d targetPose){
    Translation2d delta = targetPose.getTranslation().minus(getPose2d().getTranslation());
    return new Rotation2d(delta.getX(), delta.getY());
  }

  /**
   * Returns the length of a line from the robot's position to a specific point on the field
   * @param targetPose point to measure to
   * @return distance from robot to point
   */
  public double getDistanceFromPose(Pose2d targetPose){
    return targetPose.getTranslation().getDistance(getPose2d().getTranslation());
  }
  /**
   * gets the distance from the robot to the alliance hub
   * @return the distance from the alliance hub
   */
  public double getDistanceFromHub(){
    return getDistanceFromPose(targetHub);
  }

  public double getAngleFromHub(){
    return getPointAtPoseAngle(targetHub).getRadians();
  }

  public Pose2d getTargetHub(){
    return targetHub;
  }

  public String getTargetHubAsString(){
    if(targetHub.equals(VisionConstants.redHub)){
      return "Red";
    }
    else{
      return "Blue";
    }
  }

  public double getPointAtPoseSpeed(){
    Rotation2d desiredAngle = getPointAtPoseAngle(targetHub);
    double speed = pointToPosePID.calculate(getPose2d().getRotation().getRadians(), desiredAngle.getRadians());
    if (desiredAngle.getDegrees() > 3){
      return speed;
    }
    else{
      return 0.0;
    }
  }

  public double getPointAtPoseError(){
    return getAngleFromHub() - getPose2d().getRotation().getRadians();
  }
  /* 
  public double getPointAtSpeedUsingRelative(Pose2d target){
    double kP = 0.017;
    double poseAngle = getPose2d().relativeTo(target).getRotation().getRadians();
    return ((poseAngle + getPose2d().getRotation().getRadians()) * kP);
  }
  */
  public void updatePoseWithVision(){
    
    LimelightHelpers.PoseEstimate measurement = aimingCamera.getMegaTag2();
    if(aimingCamera.hasValidIDs()){
      swerveDrive.addVisionMeasurement(measurement.pose, measurement.timestampSeconds);
    }
    
  }

  public Vision getAimingCamera(){
    return aimingCamera;
  }

  public double getRPMFromRange(double range){
    return (VisionConstants.distanceToRPMRatio * range) + VisionConstants.baseRPM;
  }

  public Optional<Alliance> getAlliance(){
    return DriverStation.getAlliance();
  }

  

  public void configureAutoBuilder(){
    AutoBuilder.configure(
      this::getPose2d, 
      this::resetOdometry, 
      this::getChassisSpeeds, 
      this::setRobotRelativeSpeeds, 
      SwerveConstants.autoController, 
      config, 
      ()->{var alliance = DriverStation.getAlliance();
          if(alliance.isPresent()){
            return alliance.get() == DriverStation.Alliance.Red;
          } return true;
        }, 
      this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePoseWithVision();
    

    //swerveDrive.field.getObject("Vision Pose").setPose(LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.aimingLimelightName));
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    // builder.addDoubleArrayProperty("SwervePoseEstimator pose", ()->new double[]{
    //   getPose2d().getX(),
    //   getPose2d().getY(),
    //   getPose2d().getRotation().getRadians()
    // }, null);
    builder.addDoubleProperty("Pose/X", () -> getPose2d().getX(), null);    
    builder.addDoubleProperty("Pose/Y", () -> getPose2d().getY(), null);
    builder.addDoubleProperty("Pose/Rotation", () -> getPose2d().getRotation().getRadians(), null);

    builder.addDoubleProperty("Target/X", targetHub::getX, null);
    builder.addDoubleProperty("Target/Y", targetHub::getY, null);

    builder.addDoubleProperty("dist to rpm val", ()->aimingCamera.getDistToRPMVal(), null);
    builder.addDoubleProperty("distance from hub", ()->getDistanceFromHub(), null);
    builder.addDoubleProperty("angle from hub", ()->getAngleFromHub(), null);
    builder.addStringProperty("target hub", ()->getTargetHubAsString(), null);
    builder.addDoubleProperty("gyro heading", ()->swerveDrive.getPose().getRotation().getRadians(), null);
    builder.addDoubleProperty("point at pose error", ()->getPointAtPoseError(), null);
  }
}
