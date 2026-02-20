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
    swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
  }

  public double getMaxDriveSpeed(){
    return SwerveConstants.maxDriveSpeed;
  }

  public double getMaxTurnSpeed(){
    return SwerveConstants.maxTurnSpeed;
  }

  /**
   * 
   * @param translation meters per second
   * @param rotation radians per second
   * @param fieldRelative
   * @param isOpenLoop
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
    swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
  }

  public void setXMode(){
    // SwerveModuleState[] swerveXModeStates = {
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(45))};
    // swerveDrive.setModuleStates(swerveXModeStates, false);
    swerveDrive.lockPose();
  }

  public Pose2d getPose2d(){
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d pose){
    swerveDrive.resetOdometry(pose);
  }

  public ChassisSpeeds getChassisSpeeds(){
    return swerveDrive.getRobotVelocity();
  }

  public void setRobotRelativeSpeeds(ChassisSpeeds speed){
    swerveDrive.setChassisSpeeds(speed);
  }

  public double getPointAtPoseSpeed(Pose2d target){
    double kP = 0.017;
    Rotation2d targetAngle = new Rotation2d(getPose2d().getX()   - target.getX(), getPose2d().getY() - target.getY());
    return targetAngle.minus(getPose2d().getRotation()).getRadians() * kP;
  }

  public void updatePoseWithVision(){
    
    LimelightHelpers.PoseEstimate measurement = aimingCamera.getMegaTag2();
    if(aimingCamera.hasValidIDs()){
      swerveDrive.addVisionMeasurement(measurement.pose, measurement.timestampSeconds);
    }
  }

  public Vision getAimingCamera(){
    return aimingCamera;
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
          } return false;
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
    builder.addDoubleArrayProperty("SwervePoseEstimator pose", ()->new double[]{
      getPose2d().getX(),
      getPose2d().getY(),
      getPose2d().getRotation().getDegrees()
    }, null);

    builder.addDoubleProperty("dist to rpm val", ()->aimingCamera.getDistToRPMVal(), null);
  }
}
