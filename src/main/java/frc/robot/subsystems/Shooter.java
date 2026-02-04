// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.SubsystemConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX left = new TalonFX(CANIDConstants.shooterLeft);
  private TalonFX right = new TalonFX(CANIDConstants.shooterRight);
  private final double passiveTargetRPM = SubsystemConstants.maxShooterRPM / 30;
  private double targetRPM;
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
    targetRPM = RPM;
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
      left.setControl(new VelocityDutyCycle(targetRPM / 60));
      right.setControl(new VelocityDutyCycle(targetRPM / 60));
    }
    // This method will be called once per scheduler run
  }
}
