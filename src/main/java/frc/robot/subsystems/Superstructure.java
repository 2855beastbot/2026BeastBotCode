// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SubsystemConstants;

/**
 * Controls every part of the robot except the wheels. All robot actions except
 * those that
 * involve only the wheels should be handled as Commands to the
 * Superstructure and not to the robot's individual subsystems.
 */
public class Superstructure extends SubsystemBase {
  private RobotContainer container;
  // saving commonly referenced subsystems cuts down on verbosity
  private Swerve swerve;
  private IntakeWrist wrist;
  private Intake intake;
  private Shooter shooter;
  private Indexer indexer;

  /** Creates a new Superstructure. */
  public Superstructure(RobotContainer container) {
    this.container = container;
    swerve = container.getSwerve();
    wrist = container.getIntakeWrist();
    intake = container.getIntake();
    shooter = container.getShooter();
    indexer = container.getIndexer();
  }

  public Command getIdleState() {
    return Commands.run(() -> {
      // leave the intakeWrist where it is
      shooter.spin(0, false);
      indexer.spin(0);
      intake.spin(0);
      // TODO stop drive from aiming if it is, otherwise let it keep doing what it's
      // doing (pathplanner path or teleop driving)
    }).withName("Idle Superstructure State");
  }

  public Command getPickingUpState() {
    return Commands.startRun(
        () -> wrist.setTargetSetpoint(SubsystemConstants.wristOut),
        () -> {
          indexer.spin(0);
          shooter.spin(0, false);
          if (wrist.getPose() < 1.8)
            intake.spin(1);
          else
            intake.spin(0);
        },
        this).withName("Picking Up Balls Superstructure State");
  }

  public Command getShootingState() {
    return Commands.startRun(
        () -> {
          wrist.setTargetSetpoint(SubsystemConstants.wristMid);
        },
        () -> {
          shooter.setTargetRPM(swerve.inScoringArea() ? swerve.getRPMFromRange(swerve.getDistanceFromHub()) : 5000);
          indexer.spin(shooter.isAtSpeed() ? 1 : 0); // should this be debounced, or otherwise smoothed out somehow?
          if (wrist.isAtSetpoint()){
            wrist.setTargetSetpoint(wrist.getTargetSetpoint() == SubsystemConstants.wristMid ? 
              SubsystemConstants.wristIn : SubsystemConstants.wristMid);
          }
          // TODO make robot point at hub if in scoring area, also change rpm to only calculate when aiming at hub and not when feeding
        },
        this).withName("Shooting Superstructure State");
  }
}
