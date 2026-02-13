// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RPMShoot extends Command {
  /** Creates a new Shoot. */
  DoubleSupplier targetRPM;
  Shooter shooter;

  public RPMShoot(DoubleSupplier speed, Shooter ballShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    targetRPM = speed;
    shooter = ballShooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setRPMUse(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setTargetRPM(SubsystemConstants.maxShooterRPM * targetRPM.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shooter.setTargetRPM(shooter.getPassiveRPM());
    shooter.setRPMUse(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
