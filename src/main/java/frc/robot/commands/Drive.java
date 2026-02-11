// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  /** Creates a new Drive. */
  private DoubleSupplier translationX, translationY, angularRotationX;
  private Swerve swerveDrive;
  private double slowModeVal = SwerveConstants.slowModeVal;
  public Drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, Swerve drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = drivetrain;
    translationX = xSpeed;
    translationY = ySpeed;
    angularRotationX = rot;
    addRequirements(drivetrain);
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaxDriveSpeed() * slowModeVal,
                                          translationY.getAsDouble() * swerveDrive.getMaxDriveSpeed() * slowModeVal),
                        angularRotationX.getAsDouble() * swerveDrive.getMaxTurnSpeed() * slowModeVal,
                        true,
                        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
