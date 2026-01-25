// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.SubsystemConstants;

public class Intake extends SubsystemBase {
  /** Creates a new intake. */
  private SparkMax leftIntake = new SparkMax(CANIDConstants.intakeLeft,  MotorType.kBrushless);
  private SparkMax rightIntake = new SparkMax(CANIDConstants.intakeRight, MotorType.kBrushless);
  private SparkMax leftWrist = new SparkMax(CANIDConstants.intakeArmLeft, MotorType.kBrushless);
  private SparkMax rightWrist = new SparkMax(CANIDConstants.intakeArmRight, MotorType.kBrushless);
  private PIDController PIDController = new PIDController(SubsystemConstants.intakeWristKp, SubsystemConstants.intakeWristKi, SubsystemConstants.intakeWristKd);
  private AbsoluteEncoder encoder;

  private double targetSetpoint;
  public Intake() {}

  public void spin(double speed){
    leftIntake.set(speed);
    rightIntake.set(speed);
  }

  public void setTargetSetpoint(double setpoint){
    targetSetpoint = setpoint;
  }

  public double getPose(){
    return encoder.getPosition();
  }

  @Override
  public void periodic() {
    leftWrist.set(PIDController.calculate(encoder.getPosition(), targetSetpoint));
    rightWrist.set(PIDController.calculate(encoder.getPosition(), targetSetpoint));
    // This method will be called once per scheduler run
  }
}
