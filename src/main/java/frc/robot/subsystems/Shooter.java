// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.SubsystemConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX left = new TalonFX(CANIDConstants.shooterLeft);
  private TalonFX right = new TalonFX(CANIDConstants.shooterRight);
  private final double passiveTargetRPM = SubsystemConstants.maxShooterRPM / 30;
  private double targetRPS; // making this RPS instead of RPM for better internal consistency, everything outside the class is still RPM
  private boolean isUsingRPM;
  private TalonFXConfiguration config = new TalonFXConfiguration();

  public Shooter() {
    setRPMUse(true);
    spin(0);
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    left.getConfigurator().apply(config);
    right.getConfigurator().apply(config);
    
  }

  public void spin(double speed, boolean isUsingRPM){
    setRPMUse(isUsingRPM);
    spin(speed);
  }

  /**
   * sets the speed of the Shooter
   * @param speed either a traget RPM or percent power output, dependant on whether using RPM
   */
  public void spin(double speed){
    if(!isUsingRPM){
      left.setControl(new DutyCycleOut(speed));
      right.setControl(new DutyCycleOut(speed));
    }else{
      setTargetRPM(speed);
    }


  }

  /**
   * determines whether the shooter uses RPM or percent power
   * @param useRPM true uses RPM, false uses percent power
   */
  public void setRPMUse(boolean useRPM){
    isUsingRPM = useRPM;

  }

  /**
   * gets whether the shooter is using RPM
   * @return true for RPM use, false for percent power use
   */
  public boolean getRPMUse(){
    return isUsingRPM;
  }

  /**
   * sets the target RPM for the shooter
   * @param RPM the target RPM to set
   */
  public void setTargetRPM(double RPM){
    targetRPS = RPM / 60.0;
  }


  /**
   * gets the RPM of the shooters passive state
   * @return the value of passive target RPM
   */
  public double getPassiveRPM(){
    return passiveTargetRPM;
  }

  @Override
  public void periodic() {
    if(isUsingRPM){
      left.setControl(new VelocityDutyCycle(targetRPS));
      right.setControl(new VelocityDutyCycle(targetRPS));
    }
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    // open Elastic -> Add Widget -> scroll to Shooter and open the dropdown -> drag values onto dashboard
    builder.addBooleanProperty("Closed Loop", this::getRPMUse, null);
    builder.addDoubleProperty("Target RPS", () -> targetRPS, null);
    builder.addDoubleProperty("Left/Speed", left::get, null);
    builder.addDoubleProperty("Left/RPS", () -> left.getVelocity().getValueAsDouble(), null);
    builder.addDoubleProperty("Left/Current (A)", () -> left.getSupplyCurrent().getValueAsDouble(), null);
    builder.addDoubleProperty("Left/Temperature (C)", () -> left.getDeviceTemp().getValueAsDouble(), null);
    builder.addDoubleProperty("Right/Speed", right::get, null);
    builder.addDoubleProperty("Right/RPS", () -> right.getVelocity().getValueAsDouble(), null);
    builder.addDoubleProperty("Right/Current (A)", () -> right.getSupplyCurrent().getValueAsDouble(), null);
    builder.addDoubleProperty("Right/Temperature (C)", () -> right.getDeviceTemp().getValueAsDouble(), null);
  }
}
