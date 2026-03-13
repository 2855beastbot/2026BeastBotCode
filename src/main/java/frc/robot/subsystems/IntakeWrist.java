// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.SubsystemConstants;

public class IntakeWrist extends SubsystemBase {
  /** Creates a new IntakeWrist. */
  private SparkMax leftWrist = new SparkMax(CANIDConstants.intakeArmLeft, MotorType.kBrushless);
  private SparkMax rightWrist = new SparkMax(CANIDConstants.intakeArmRight, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();
  private SparkAbsoluteEncoder encoder;
  private double targetSetpoint;
  private boolean isOpenLoop;
  private PIDController pidController = new PIDController(SubsystemConstants.intakeWristKp, SubsystemConstants.intakeWristKi, SubsystemConstants.intakeWristKd);
  
  public IntakeWrist() {
    encoder = rightWrist.getAbsoluteEncoder();
    config.absoluteEncoder.inverted(true);
    setTargetSetpoint(getPose());
    config.closedLoop.pid(SubsystemConstants.intakeWristKp, SubsystemConstants.intakeWristKi, SubsystemConstants.intakeWristKd);
    //config.closedLoop.feedForward.kCos(0);
    rightWrist.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    pidController.enableContinuousInput(0, Math.PI * 2);
    SmartDashboard.putData("Intake PID controller", pidController);
    
    // make left follow right, now everything sent to right, left will do automatically
    leftWrist.configure(new SparkMaxConfig().follow(rightWrist, true), null, PersistMode.kNoPersistParameters);
  }

  public void setTargetSetpoint(double setpoint){
    isOpenLoop = false;
    targetSetpoint = setpoint;
  }

  public double getTargetSetpoint(){
    return targetSetpoint;
  }

   public double getPose(){
    return encoder.getPosition();
  }

  public void moveWrist(double speed){
    isOpenLoop = true;
    rightWrist.set(-speed);
  }

  public boolean isOpenLoop(){
    return isOpenLoop;
  }

  public boolean isAtSetpoint(){
    return rightWrist.getClosedLoopController().isAtSetpoint();
  }

  public double getOutputCurrent(){
    return rightWrist.getOutputCurrent();
  }

  public void zeroEncoders(){
    leftWrist.getEncoder().setPosition(0.0);
    rightWrist.getEncoder().setPosition(0.0);
  }

   public void runPID(){
    double angle = encoder.getPosition();
    if(angle > 2.15){
      angle = angle - (Math.PI * 2);
    }
    rightWrist.set(-pidController.calculate(angle, targetSetpoint));
  }

  @Override
  public void periodic() {
    if(!isOpenLoop){
      runPID();
    }
    
    // This method will be called once per scheduler run
  }

@Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder); 
    builder.addBooleanProperty("Open Loop", this::isOpenLoop, null);
    builder.addDoubleProperty("Position", encoder::getPosition, null);
    builder.addDoubleProperty("Target Pos", this::getTargetSetpoint, null);
    builder.addDoubleProperty("Left Wrist/Speed", leftWrist::get, null);
    builder.addDoubleProperty("Left Wrist/Output", leftWrist::getAppliedOutput, null);
    builder.addDoubleProperty("Left Wrist/Current (A)", leftWrist::getOutputCurrent, null);
    builder.addDoubleProperty("Left Wrist/Temperature (C)", leftWrist::getMotorTemperature, null);
    builder.addDoubleProperty("Right Wrist/Speed", rightWrist::get, null);
    builder.addDoubleProperty("Right Wrist/Output", rightWrist::getAppliedOutput, null);
    builder.addDoubleProperty("Right Wrist/Current (A)", rightWrist::getOutputCurrent, null);
    builder.addDoubleProperty("Right Wrist/Temperature (C)", rightWrist::getMotorTemperature, null);
  }

}
