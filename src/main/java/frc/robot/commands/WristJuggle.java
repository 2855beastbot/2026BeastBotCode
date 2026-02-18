// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristJuggle extends Command {
  /** Creates a new WristJuggle. */
  private Intake intake;
  public int i;
  public WristJuggle(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.moveWrist(0.5);
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setTargetSetpoint(SubsystemConstants.wristOut);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(intake.getPose() > (SubsystemConstants.wristIn - 0.0125));
    return false;
  }
}
