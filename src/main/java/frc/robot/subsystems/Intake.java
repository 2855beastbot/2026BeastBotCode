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

  public Intake() {
    encoder = rightWrist.getAbsoluteEncoder();
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
    // TODO: make sure encoder is configured so that readings match specification in doc comment (zero position, positive direction) 
    targetSetpoint = (setpoint / 360) * SubsystemConstants.wristGearboxCoef;  //convert degrees to rotations, gear ratios
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
    rightWrist.set(speed);
  }

  /**
   * runs a single motor at 30% speed
   * @param motorID the CAN ID of the motor to run
   */
  public void runMotor(int motorID){
    if(motorID == rightWrist.getDeviceId()){
      rightWrist.set(0.3);
    }else if(motorID == leftWrist.getDeviceId()){
      leftWrist.set(0.3);
    }else if(motorID == leftIntake.getDeviceId()){
      leftIntake.set(0.3);
    }else if(motorID == rightIntake.getDeviceId()){
      rightIntake.set(0.3);
    }
  }

  @Override
  public void periodic() {
    leftWrist.set(PIDController.calculate(encoder.getPosition(), targetSetpoint));
    rightWrist.set(PIDController.calculate(encoder.getPosition(), targetSetpoint));
    // This method will be called once per scheduler run
  }
}
