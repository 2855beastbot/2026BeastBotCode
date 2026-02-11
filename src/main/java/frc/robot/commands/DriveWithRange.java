// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveWithRange extends Command {
  /** Creates a new DriveWithRange. */
  private DoubleSupplier xSpeed;
  private Vision aimingCamera;
  private Swerve drivetrain;
  private double range;
  /**
   * drives the robot to a certain range from valid apriltag while also aiming at it. the driver can still control lateral movement, which will result in an arc
   * @param leftX the lateral speeds
   * @param swerve the drivetrain subsystem
   * @param range the distance to maintain between the robot and the Apriltag, in meters
   */
  public DriveWithRange(DoubleSupplier leftX, Swerve swerve, double range) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    xSpeed = leftX;
    drivetrain = swerve;
    aimingCamera = swerve.getAimingCamera();
    this.range = range;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aimingCamera.setValidIDs(VisionConstants.targetingIDs);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(
      new Translation2d(
        xSpeed.getAsDouble() * SwerveConstants.maxDriveSpeed * SwerveConstants.slowModeVal,
        aimingCamera.rangeWithVision(range) * SwerveConstants.slowModeVal
      ), 
      aimingCamera.aimWithVision() * SwerveConstants.slowModeVal, 
      true, 
      true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    aimingCamera.setValidIDs(VisionConstants.allIDs);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
