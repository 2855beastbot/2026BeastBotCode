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
public class DriveWithAim extends Command {
  /** Creates a new DriveWithAim. */
  private DoubleSupplier xSpeed, ySpeed;
  private Vision aimingCamera;
  private Swerve drivetrain;
  /**
   * aims the drivetrain at valid Apriltags, driver still controls translational movement
   * @param leftY the forward speed of the robot
   * @param leftX the sideways speed of the robot
   * @param swerve the drivetrain class
   */
  public DriveWithAim(DoubleSupplier leftY, DoubleSupplier leftX, Swerve swerve) {
    addRequirements(swerve);
    xSpeed = leftX;
    ySpeed = leftY;
    drivetrain = swerve;
    aimingCamera = swerve.getAimingCamera();
    // Use addRequirements() here to declare subsystem dependencies.
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
        ySpeed.getAsDouble() * SwerveConstants.maxDriveSpeed * SwerveConstants.slowModeVal), 
      aimingCamera.aimWithVision() * SwerveConstants.maxTurnSpeed * SwerveConstants.slowModeVal, 
      false, 
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
