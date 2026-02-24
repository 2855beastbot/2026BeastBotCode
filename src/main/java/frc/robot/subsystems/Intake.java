// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.SubsystemConstants;

public class Intake extends SubsystemBase {
  /** Creates a new intake. */
  private SparkMax leftIntake = new SparkMax(CANIDConstants.intakeLeft,  MotorType.kBrushless);
  private SparkMax rightIntake = new SparkMax(CANIDConstants.intakeRight, MotorType.kBrushless);
  
  
  
  

  public Intake() { 
  }

  /**
   * runs the intake wheels
   * @param speed the percent power to the wheels, from 0-1
   */
  public void spin(double speed){
    leftIntake.set(-speed);
    //if(getPose() < 1.8)
      rightIntake.set(-speed);
  }

  /**
   * Sets the target angle for the intake to hold measured in radians up from horizontal ("out") position
   * @param setpoint target angle in degrees
   */
  

  

  /**
   * gets the encoders current position
   * @return the encoder position
   */
 

  /**
   * open loop for both wrist motors
   * @param speed the power to feed the motors, from 0-1
   */
  

 

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    // open Elastic -> Add Widget -> scroll to Intake and open the dropdown -> drag values onto dashboard
    
    builder.addDoubleProperty("Intake Speed (left)", leftIntake::get, null);
    builder.addDoubleProperty("Intake Speed (right)", rightIntake::get, null);
  }

}
