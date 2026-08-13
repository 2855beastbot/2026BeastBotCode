// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

/**
 * Controls every part of the robot except the wheels. All robot actions that do
 * not involve only the wheels should be handled as Commands to the
 * Superstructure and not to the robot's individual subsystems.
 */
public class Superstructure extends SubsystemBase {
  private RobotContainer container;

  /** Creates a new Superstructure. */
  public Superstructure(RobotContainer container) {
    this.container = container;
  }

  public Command getIdleState() {
    return defer(() -> new InstantCommand(() -> {
      // leave the intakeWrist where it is
      container.getShooter().spin(0, false);
      container.getIndexer().spin(0);
      container.getIntake().spin(0);
      // TODO stop drive from aiming if it is, otherwise let it keep doing what it's
      // doing
    }));
  }

  public Command getPickingUpState() {
    // TODO add code
    return Commands.none();
  }

  public Command getShootingState() {
    // TODO add code
    return Commands.none();
  }
}
