// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootWithRange extends Command {
  /** Creates a new ShootWithRange. */
  private Vision aimingCamera;
  private Shooter ballShooter;
  public ShootWithRange(Vision camera, Shooter shooter) {
    addRequirements(camera, shooter);
    aimingCamera = camera;
    ballShooter = shooter;
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
    if(aimingCamera.hasValidIDs())
    ballShooter.setTargetRPM(aimingCamera.calculateRPMFromRange());
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
