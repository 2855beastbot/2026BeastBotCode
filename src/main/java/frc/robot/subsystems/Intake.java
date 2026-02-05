// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
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
  private boolean isOpenLoop;
  

  public Intake() {
    encoder = rightWrist.getAbsoluteEncoder();
    
    // make left follow right, now everything sent to right, left will do automatically
    leftWrist.configure(new SparkMaxConfig().follow(rightWrist, true), null, PersistMode.kPersistParameters);
  }

  /**
   * runs the intake wheels
   * @param speed the percent power to the wheels, from 0-1
   */
  public void spin(double speed){
    speed *= 0.75;
    leftIntake.set(speed);
    rightIntake.set(speed);
  }

  /**
   * Sets the target angle for the intake to hold measured in degrees up from horizontal ("out") position
   * @param setpoint target angle in degrees
   */
  public void setTargetSetpoint(double setpoint){
    isOpenLoop = true;
    // TODO: make sure encoder is configured so that readings match specification in doc comment (zero position, positive direction) 
    targetSetpoint = (setpoint / 360) * SubsystemConstants.wristGearboxCoef;  //convert degrees to rotations, gear ratios
  }

  public double getTargetSetpoint(){
    return targetSetpoint;
  }

  /**
   * gets the encoders current position
   * @return the encoder position, converted to intake rotations
   */
  public double getPose(){
    return encoder.getPosition() / SubsystemConstants.wristGearboxCoef; // converts from encoder to intake rotations
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
  public void runMotor(int motorID){
    if(motorID == rightWrist.getDeviceId()){
      rightWrist.set(0.3);
    }else if(motorID == leftIntake.getDeviceId()){
      leftIntake.set(0.3);
    }else if(motorID == rightIntake.getDeviceId()){
      rightIntake.set(0.3);
    }
  }

  public void setOpenLoop(boolean openLoop){
    isOpenLoop = openLoop;
  }

  public boolean isOpenLoop(){
    return isOpenLoop;
  }

  public void zeroEncoders(){
    leftWrist.getAlternateEncoder().setPosition(0.0);
    rightWrist.getAlternateEncoder().setPosition(0.0);
  }

  @Override
  public void periodic() {
    if(!isOpenLoop){
      rightWrist.set(PIDController.calculate(encoder.getPosition(), targetSetpoint) + 0.1);
    }
    
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    // open Elastic -> Add Widget -> scroll to Intake and open the dropdown -> drag values onto dashboard
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
    builder.addDoubleProperty("Intake Speed (left)", leftIntake::get, null);
    builder.addDoubleProperty("Intake Speed (right)", rightIntake::get, null);
  }

}
