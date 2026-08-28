// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;

public class Intake extends SubsystemBase {
  private TalonFX leftIntake = new TalonFX(CANIDConstants.intakeLeft);
  private TalonFX rightIntake = new TalonFX(CANIDConstants.intakeRight);
  private TalonFXConfiguration leftConfig = new TalonFXConfiguration();
  private TalonFXConfiguration rightConfig = new TalonFXConfiguration();
  
  
  private IntakeWrist intakeWrist;
  public Intake(IntakeWrist wrist) {

    leftConfig.MotorOutput.Inverted = leftConfig.MotorOutput.Inverted.Clockwise_Positive;
    leftConfig.CurrentLimits.SupplyCurrentLimit = 30.0;

    
    rightConfig.CurrentLimits.SupplyCurrentLimit = 30.0;

    leftIntake.getConfigurator().apply(leftConfig);
    rightIntake.getConfigurator().apply(rightConfig);
    intakeWrist = wrist; 
  }

  /**
   * runs the intake wheels
   * @param speed the percent power to the wheels, from 0-1
   */
  public void spin(double speed){
    if(intakeWrist.getPose() < 0.5) {
      //leftIntake.set(-speed);
      //rightIntake.set(-speed);
      
      rightIntake.set(-speed);
    }
    else{
      
      rightIntake.set(0);
    }
    leftIntake.set(-speed);
      
  }

  public Command intake(DoubleSupplier speed){
    return run(() -> {
      rightIntake.set(-speed.getAsDouble());
      leftIntake.set(-speed.getAsDouble());
    });
  }

  public Command intakeOuterRollerOnly(DoubleSupplier speed){
    return run(() -> {
      rightIntake.set(0);
      leftIntake.set(-speed.getAsDouble());
    });
  }
 

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    // open Elastic -> Add Widget -> scroll to Intake and open the dropdown -> drag values onto dashboard
    
    builder.addDoubleProperty("Intake Speed (left)", leftIntake::get, null);
    builder.addDoubleProperty("Intake Speed (right)", rightIntake::get, null);
  }

}
