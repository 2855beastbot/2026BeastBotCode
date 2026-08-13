// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simutils;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

public class RobotSim extends SubsystemBase {
  /** Creates a new RobotSim. */
  public RobotSim() {
    SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(2,2)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic(){
    // Get the positions of the fuel (both on the field and in the air)
    Pose3d[] fuelPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
    // Publish to telemetry using AdvantageKit
    Logger.recordOutput("FieldSimulation/FuelPositions", fuelPoses);
  }
}
