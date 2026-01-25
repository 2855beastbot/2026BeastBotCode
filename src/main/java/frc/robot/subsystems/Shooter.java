// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX left = new TalonFX(CANIDConstants.shooterLeft);
  private TalonFX right = new TalonFX(CANIDConstants.shooterRight);
  private final double passiveTargetRPM = 60;
  private double targetRPM;
  private boolean isUsingRPM;
  public Shooter() {
    setTargetRPM(passiveTargetRPM);
  }

  public void spin(double speed){
    if(!isUsingRPM){
      left.setControl(new DutyCycleOut(speed));
      right.setControl(new DutyCycleOut(speed));
    }


  }

  public void setRPMUse(boolean useRPM){
    isUsingRPM = (useRPM) ? true : false;

  }

  public boolean getRPMUse(){
    return isUsingRPM;
  }

  public void setTargetRPM(double RPM){
    targetRPM = RPM;
  }

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
