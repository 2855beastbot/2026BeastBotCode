// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.security.MessageDigest;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AllianceInfo;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.RawFiducial;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {
  private enum TeleopState {
    FREE,
    AIMING
  }

  /** Creates a new Swerve. */
  private SwerveDrive swerveDrive;
  private RobotConfig config;
  private Vision aimingCamera = new Vision(VisionConstants.aimingLimelightName, VisionConstants.aimingConfig);
  private final PIDController pointToPosePID = new PIDController(20.0, 0.0, 0.5);
  private TeleopState currentState;

  public Swerve() {

    Pose2d startingPose;
    if (RobotBase.isSimulation()) {
      // chooses default position based on chosen target hub, vision should override
      // this
      startingPose = AllianceInfo.isBlue() ? new Pose2d(new Translation2d(3, 4),
          Rotation2d.fromDegrees(0))
          : new Pose2d(new Translation2d(13, 4),
              Rotation2d.fromDegrees(180));
    } else {
      startingPose = new Pose2d();
    }

    double maximumSpeed = Units.feetToMeters(4.5);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed, startingPose);
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      DataLogManager.log("Could not read PathPlanner config file.");
      e.printStackTrace();
    }
    configureAutoBuilder();
    swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(1.5, 1.5, 9999999)); // higher
                                                                                                          // number
                                                                                                          // means less
                                                                                                          // trust
    // reiously0.7,0.7,9999999
    pointToPosePID.enableContinuousInput(-Math.PI, Math.PI);
    pointToPosePID.setTolerance(2.0);

    currentState = TeleopState.FREE;
  }

  public double getMaxDriveSpeed() {
    return SwerveConstants.maxDriveSpeed;
  }

  public double getMaxTurnSpeed() {
    return SwerveConstants.maxTurnSpeed;
  }

  /**
   * Primary method for driving robot
   * 
   * @param translation   meters per second
   * @param rotation      radians per second
   * @param fieldRelative
   * @param isOpenLoop
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
  }

  /**
   * Method for driving based on controller inputs. Values are processed under the
   * assumption that they come from a gamepad and may not function correctly when
   * coming from other sources.
   * 
   * @param controllerX        value for movement along the field's X axis,
   *                           usually the Y axis of the left stick
   * @param controllerY        value for movement along the field's Y axis,
   *                           usually the X axis of the left stick
   * @param controllerRotation value for robot's rotation, usually the X axis on
   *                           the right stick
   */
  public void teleopDrive(double controllerX, double controllerY, double controllerRotation) {
    double rotationSpeed = (isAiming() && inScoringArea()) ? getPointAtPoseSpeed()
        : MathUtil.applyDeadband(controllerRotation, 0.1) * getMaxTurnSpeed() * -1 * SwerveConstants.slowModeVal;

    int flipCoeff = AllianceInfo.isBlue() ? -1 : 1; // position inputs need to be reversed depending on which side of
                                                    // the field the alliance is on

    double xSpeed = MathUtil.applyDeadband(controllerX, 0.1) * getMaxDriveSpeed() * flipCoeff
        * SwerveConstants.slowModeVal;
    double ySpeed = MathUtil.applyDeadband(controllerY, 0.1) * getMaxDriveSpeed() * flipCoeff
        * SwerveConstants.slowModeVal;

    if (isAiming()
        && xSpeed == 0
        && ySpeed == 0
        && rotationSpeed == 0) {
      setXMode();
    } else {
      swerveDrive.drive(new Translation2d(xSpeed, ySpeed), rotationSpeed, true, true);
    }
  }

  /**
   * Drive robot while pointing at alliance hub
   * 
   * @param translation x and y speeds to drive at
   */
  public void drivePose(Translation2d translation) {
    Rotation2d desiredAngle = getPointAtPoseAngle(AllianceInfo.getTargetHubPos());
    double rotationSpeed = pointToPosePID.calculate(
        getPose2d().getRotation().getRadians(),
        desiredAngle.getRadians());
    swerveDrive.drive(translation, rotationSpeed, true, false);
  }

  public void setXMode() {
    swerveDrive.lockPose();
  }

  public Pose2d getPose2d() {
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  /**
   * resets the pose of the robot to the passed in pose, rotating by 180 if in red
   * alliance
   * 
   * @param pose the pose to set to
   */
  public void resetOdometryWithAlliance(Pose2d pose) {
    if (AllianceInfo.isBlue()) {
      swerveDrive.resetOdometry(pose);
    } else {
      swerveDrive
          .resetOdometry(new Pose2d(pose.getTranslation(), new Rotation2d(pose.getRotation().getRadians() + Math.PI)));
    }
  }

  public ChassisSpeeds getChassisSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public void setRobotRelativeSpeeds(ChassisSpeeds speed) {
    swerveDrive.setChassisSpeeds(speed);
  }

  /**
   * Returns the length of a line from the robot's position to a specific point on
   * the field
   * 
   * @param targetPose point to measure to
   * @return distance from robot to point
   */
  public double getDistanceFromPose(Pose2d targetPose) {
    return targetPose.getTranslation().getDistance(getPose2d().getTranslation());
  }

  /**
   * gets the distance from the robot to the alliance hub
   * 
   * @return the distance from the alliance hub
   */
  public double getDistanceFromHub() {
    return getDistanceFromPose(AllianceInfo.getTargetHubPos());
  }

  public double getAngleFromHub() {
    return getPointAtPoseAngle(AllianceInfo.getTargetHubPos()).getRadians();
  }

  /**
   * Returns the angle of a line pointing from the robot's position to a specific
   * position on the field
   * 
   * @param targetPose position to point at
   * @return angle from robot to target position
   */
  public Rotation2d getPointAtPoseAngle(Pose2d targetPose) {
    Translation2d delta = targetPose.getTranslation().minus(getPose2d().getTranslation());
    return new Rotation2d(delta.getX(), delta.getY()).plus(new Rotation2d(Math.PI));
  }

  public double getPointAtPoseSpeed() {
    Rotation2d desiredAngle = getPointAtPoseAngle(AllianceInfo.getTargetHubPos());
    double speed = pointToPosePID.calculate(getPose2d().getRotation().getRadians(), desiredAngle.getRadians());
    return speed;
    // if (desiredAngle.getDegrees() > 3) { // PIDControllers already do this
    // return speed;
    // } else {
    // return 0.0;
    // }
  }

  public double getPointAtPoseError() {
    return getAngleFromHub() - getPose2d().getRotation().getRadians();
  }

  // ...existing code...
  double minimumXYstd = 0.3;
  double maximumXYstd = 0.7;
  double deviationToReject = 0.7;

  public void setVisionStdDynamic(Pose2d newPose2dFromVision) {
    RawFiducial[] tagsSeen = LimelightHelpers.getRawFiducials(aimingCamera.getName());

    if (tagsSeen == null || tagsSeen.length == 0) {
      // No tags — don't trust vision at all
      swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(9999999, 9999999, 9999999));
      return;
    }

    // Find average distance to all visible tags
    double totalDistance = 0.0;
    for (RawFiducial tag : tagsSeen) {
      totalDistance += tag.distToCamera;
    }
    double avgDistance = totalDistance / tagsSeen.length;

    // When disabled, trust vision heavily so starting pose converges quickly
    if (DriverStation.isDisabled()) {
      // Scale lightly with distance but keep very low std devs
      double disabledStd = 0.1 * avgDistance / tagsSeen.length;
      disabledStd = Math.max(0.05, Math.min(disabledStd, 0.5));
      swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(disabledStd, disabledStd, 9999999));
      return;
    }

    // --- Normal enabled behavior below ---

    // Squaring the distance better represents real-world vision noise
    double newXYstd = (0.5 * Math.pow(avgDistance, 2)) / tagsSeen.length;

    // Soft-clamp the pose jump based on tag count and distance
    double poseJump = newPose2dFromVision.getTranslation().getDistance(getPose2d().getTranslation());
    if (poseJump > deviationToReject) {
      if (tagsSeen.length >= 2 && avgDistance < 2.0) {
        // We see multiple tags up close. We probably actually got pushed.
        // Don't penalize.
      } else {
        newXYstd *= 3.0; // Heavily distrust large jumps on single/far tags
      }
    }

    // Clamp to reasonable range
    newXYstd = Math.max(minimumXYstd, Math.min(newXYstd, maximumXYstd));

    swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(newXYstd, newXYstd, 9999999));
  }

  public void updatePoseWithVision() {
    LimelightHelpers.PoseEstimate measurement = aimingCamera.getMegaTag2(swerveDrive.getPose());
    if (aimingCamera.hasValidIDs()) {
      // setVisionStdDynamic(measurement.pose);
      swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));

      swerveDrive.addVisionMeasurement(measurement.pose, measurement.timestampSeconds);
    }
  }

  public Vision getAimingCamera() {
    return aimingCamera;
  }

  public double getRPMFromRange(double range) {
    return (VisionConstants.distanceToRPMRatio * range) + VisionConstants.baseRPM;
  }

  public boolean inScoringArea() {
    if (AllianceInfo.isBlue()) {
      return getPose2d().getX() < VisionConstants.blueHub.getX();
    } else {
      return getPose2d().getX() > VisionConstants.redHub.getX();
    }
  }

  public void startAiming() {
    currentState = TeleopState.AIMING;
  }

  public void cancelAiming() {
    currentState = TeleopState.FREE;
  }

  public boolean isAiming() {
    return currentState == TeleopState.AIMING;
  }

  public void configureAutoBuilder() {
    AutoBuilder.configure(
        this::getPose2d,
        this::resetOdometry,
        this::getChassisSpeeds,
        this::setRobotRelativeSpeeds,
        SwerveConstants.autoController,
        config,
        AllianceInfo::isRed,
        this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePoseWithVision();

    // swerveDrive.field.getObject("Vision
    // Pose").setPose(LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.aimingLimelightName));

    // DataLogManager.log("Swerve state: " + currentState + ", in scoring area: " +
    // inScoringArea());
    // DataLogManager.log("Robot pose: " + getPose2d().getX() + ", " +
    // getPose2d().getY());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // builder.addDoubleArrayProperty("SwervePoseEstimator pose", ()->new double[]{
    // getPose2d().getX(),
    // getPose2d().getY(),
    // getPose2d().getRotation().getRadians()
    // }, null);

    // Field2d theField(Field2d) Shuffleboard.getTab("field").
    builder.addDoubleProperty("Pose/X", () -> getPose2d().getX(), null);
    builder.addDoubleProperty("Pose/Y", () -> getPose2d().getY(), null);
    builder.addDoubleProperty("Pose/Rotation", () -> getPose2d().getRotation().getRadians(), null);

    builder.addDoubleProperty("Target/X", () -> AllianceInfo.getTargetHubPos().getX(), null);
    builder.addDoubleProperty("Target/Y", () -> AllianceInfo.getTargetHubPos().getY(), null);

    builder.addDoubleProperty("dist to rpm val", () -> aimingCamera.getDistToRPMVal(), null);
    builder.addDoubleProperty("distance from hub", () -> getDistanceFromHub(), null);
    builder.addDoubleProperty("angle from hub", () -> getAngleFromHub(), null);
    builder.addStringProperty("target hub", () -> AllianceInfo.getAlliance().toString(), null);
    builder.addDoubleProperty("gyro heading", () -> swerveDrive.getPose().getRotation().getRadians(), null);
    builder.addDoubleProperty("point at pose error", () -> getPointAtPoseError(), null);

    builder.addBooleanProperty("in scoring area", this::inScoringArea, null);
  }
}
