// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SubsystemConstants;

/**
 * Controls every part of the robot except the wheels. All robot actions except
 * driving should be handled as Commands to the Superstructure and not to the
 * robot's individual subsystems. Driving is handled by PathPlanner during auto 
 * and the drive's own commands during teleop, but the Superstructure may request 
 * actions of the Swervedrive via methods like startAiming() and cancelAiming().
 */
public class Superstructure extends SubsystemBase {
  private RobotContainer container;
  // saving commonly referenced subsystems cuts down on typing
  private Swerve swerve;
  private IntakeWrist wrist;
  private Intake intake;
  private Shooter shooter;
  private Indexer indexer;

  private final Command idleState;
  private final Command pickingUpState;
  private final Command shootingState;

  /** Creates a new Superstructure. */
  public Superstructure(RobotContainer container) {
    this.container = container;
    swerve = container.getSwerve();
    wrist = container.getIntakeWrist();
    intake = container.getIntake();
    shooter = container.getShooter();
    indexer = container.getIndexer();

    idleState = startRun(
      () -> swerve.cancelAiming(),
      () -> {
        // leave the intakeWrist where it is
        indexer.spin(0);
        shooter.spin(0, false);
        intake.spin(0);
      }).withName("Idle Superstate");

    pickingUpState = startRun(
      () -> {
        wrist.setTargetSetpoint(SubsystemConstants.wristOut);
        swerve.cancelAiming();
      },
      () -> {
        indexer.spin(0);
        shooter.spin(0, false);
        intake.spin(wrist.getPose() < 1.8 ? 1 : 0);
      }).withName("Picking Up Balls Superstate");

    shootingState = startRun(
    () -> {
      wrist.setTargetSetpoint(SubsystemConstants.wristMid);
      swerve.startAiming();
    },
    () -> {
      if(swerve.inScoringArea()){
        shooter.shootForDistance(swerve.getDistanceFromHub());
      } else {
        shooter.setTargetRPM(5000);
      }
      indexer.spin(shooter.isAtSpeed() && swerve.pointedAtTarget() ? 1 : 0); // should this be debounced, or otherwise smoothed out somehow?
      if (wrist.isAtSetpoint()) {
        wrist
            .setTargetSetpoint(wrist.getTargetSetpoint() == SubsystemConstants.wristMid ? SubsystemConstants.wristIn
                : SubsystemConstants.wristMid);
      }
    }).withName("Shooting Superstate");
  }

  public Command idleState() {
    return idleState;
  }

  public Command pickingUpState() {
    return pickingUpState;
  }

  public Command shootingState() {
    return shootingState;
  }
}
