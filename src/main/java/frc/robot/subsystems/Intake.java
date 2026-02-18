// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.SubsystemConstants;

public class Intake extends SubsystemBase {
  /** Creates a new intake. */
  private SparkMax leftIntake = new SparkMax(CANIDConstants.intakeLeft,  MotorType.kBrushless);
  private SparkMax rightIntake = new SparkMax(CANIDConstants.intakeRight, MotorType.kBrushless);
  private SparkMax leftWrist = new SparkMax(CANIDConstants.intakeArmLeft, MotorType.kBrushless);
  private SparkMax rightWrist = new SparkMax(CANIDConstants.intakeArmRight, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();
  private MAXMotionConfig trapezoidalPID = config.closedLoop.maxMotion;
  private PIDController pidController = new PIDController(SubsystemConstants.intakeWristKp, SubsystemConstants.intakeWristKi, SubsystemConstants.intakeWristKd);
  private SparkAbsoluteEncoder encoder;
  private double targetSetpoint;
  private boolean isOpenLoop;
  

  public Intake() {
    encoder = rightWrist.getAbsoluteEncoder();
    config.absoluteEncoder.inverted(true);
    setTargetSetpoint(getPose());
    config.closedLoop.pid(SubsystemConstants.intakeWristKp, SubsystemConstants.intakeWristKi, SubsystemConstants.intakeWristKd);
    trapezoidalPID.maxAcceleration(2);
    trapezoidalPID.cruiseVelocity(5);
    trapezoidalPID.allowedProfileError(0.1);
    //config.closedLoop.feedForward.kCos(0);
    rightWrist.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    pidController.disableContinuousInput();
    

    
    // make left follow right, now everything sent to right, left will do automatically
    leftWrist.configure(new SparkMaxConfig().follow(rightWrist, true), null, PersistMode.kNoPersistParameters);
  }

  /**
   * runs the intake wheels
   * @param speed the percent power to the wheels, from 0-1
   */
  public void spin(double speed){
    leftIntake.set(speed);
    rightIntake.set(speed);
  }

  /**
   * Sets the target angle for the intake to hold measured in degrees up from horizontal ("out") position
   * @param setpoint target angle in degrees
   */
  public void setTargetSetpoint(double setpoint){
    isOpenLoop = false;
    // TODO: make sure encoder is configured so that readings match specification in doc comment (zero position, positive direction) 
    targetSetpoint = setpoint;
  }

  public double getTargetSetpoint(){
    return targetSetpoint;
  }

  /**
   * gets the encoders current position
   * @return the encoder position, converted to intake rotations
   */
  public double getAbsPose(){
    return encoder.getPosition(); // converts from encoder to intake rotations
  }

  public double getPose(){
    return encoder.getPosition();
  }

  /**
   * open loop for both wrist motors
   * @param speed the power to feed the motors, from 0-1
   */
  public void moveWrist(double speed){
    rightWrist.set(-speed);
  }

  /**
   * runs a single motor at 30% speed
   * @param motorID the CAN ID of the motor to run
   */
  
  public void setOpenLoop(boolean openLoop){
    isOpenLoop = openLoop;
  }

  public boolean isOpenLoop(){
    return isOpenLoop;
  }

  public boolean isAtSetpoint(){
    return rightWrist.getClosedLoopController().isAtSetpoint();
  }

  public void zeroEncoders(){
    leftWrist.getEncoder().setPosition(0.0);
    rightWrist.getEncoder().setPosition(0.0);
    
    
  }
  public SparkMax getWrist(){
    return leftWrist;
  }

  @Override
  public void periodic() {
    if(!isOpenLoop){
      rightWrist.set(pidController.calculate(encoder.getPosition(), targetSetpoint));
    }
    
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    // open Elastic -> Add Widget -> scroll to Intake and open the dropdown -> drag values onto dashboard
    builder.addBooleanProperty("Open Loop", this::isOpenLoop, null);
    builder.addDoubleProperty("Position", encoder::getPosition, null);
    builder.addDoubleProperty("Motor Pos", ()->leftWrist.getEncoder().getPosition(), null);
    builder.addDoubleProperty("Target Pos", this::getTargetSetpoint, null);
    builder.addDoubleProperty("Left Wrist/Speed", leftWrist::get, null);
    builder.addDoubleProperty("Left Wrist/Output", leftWrist::getAppliedOutput, null);
    builder.addDoubleProperty("Left Wrist/Current (A)", leftWrist::getOutputCurrent, null);
    builder.addDoubleProperty("Left Wrist/Temperature (C)", leftWrist::getMotorTemperature, null);
    builder.addDoubleProperty("Right Wrist/Speed", rightWrist::get, null);
    builder.addDoubleProperty("Right Wrist/Output", rightWrist::getAppliedOutput, null);
    builder.addDoubleProperty("Right Wrist/Current (A)", rightWrist::getOutputCurrent, null);
    builder.addDoubleProperty("Right Wrist/Temperature (C)", rightWrist::getMotorTemperature, null);
    builder.addDoubleProperty("Intake Speed (left)", leftIntake::get, null);
    builder.addDoubleProperty("Intake Speed (right)", rightIntake::get, null);
  }

}
