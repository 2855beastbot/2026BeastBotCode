// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;


public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private SparkMax indexer = new SparkMax(CANIDConstants.indexer, MotorType.kBrushless);
  public Indexer() {
    setDefaultCommand(stop());
  }

  public Command spin(DoubleSupplier speed){
    return run(() -> indexer.set(speed.getAsDouble())).withName("Spinning");
  }

  public Command spin(double speed){
    return run(() -> indexer.set(speed)).withName("Spinning");
  }

  public Command stop(){
    return run(() -> indexer.set(0)).withName("Stopped");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
