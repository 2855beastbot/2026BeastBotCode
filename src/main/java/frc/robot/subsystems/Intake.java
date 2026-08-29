// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;

public class Intake extends SubsystemBase {
  private TalonFX leftIntake = new TalonFX(CANIDConstants.intakeLeft);
  private TalonFX rightIntake = new TalonFX(CANIDConstants.intakeRight);
  private TalonFXConfiguration leftConfig = new TalonFXConfiguration();
  private TalonFXConfiguration rightConfig = new TalonFXConfiguration();

  public Intake() {

    leftConfig.MotorOutput.Inverted = leftConfig.MotorOutput.Inverted.Clockwise_Positive;
    leftConfig.CurrentLimits.SupplyCurrentLimit = 30.0;

    rightConfig.CurrentLimits.SupplyCurrentLimit = 30.0;

    leftIntake.getConfigurator().apply(leftConfig);
    rightIntake.getConfigurator().apply(rightConfig);
  }

  public Command intakeSafely(DoubleSupplier speed, BooleanSupplier safeToSpinBoth) {
    return Commands.either(intake(speed), intakeOneRollerOnly(speed), safeToSpinBoth).withName("Intake with check");
  }

  private Command intake(DoubleSupplier speed) {
    return runEnd(
        () -> {
          rightIntake.set(-speed.getAsDouble());
          leftIntake.set(-speed.getAsDouble());
        },
        () -> {
          rightIntake.set(0);
          leftIntake.set(0);
        })
        .withName("Intaking");
  }

  private Command intakeOneRollerOnly(DoubleSupplier speed) {
    return runEnd(
        () -> {
          rightIntake.set(0);
          leftIntake.set(-speed.getAsDouble());
        },
        () -> {
          leftIntake.set(0);
        })
        .withName("Intaking One Roller Only");
  }

  /**
   * Use this command during auto to ensure the intake rollers start spinning in
   * time
   * 
   * @return
   */
  public Command spamWheels() {
    return intake(() -> 1).withTimeout(0.1).repeatedly().asProxy().withName("SPAM SPAM SPAM FUEL AND SPAM");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // open Elastic -> Add Widget -> scroll to Intake and open the dropdown -> drag
    // values onto dashboard

    builder.addDoubleProperty("Intake Speed (left)", leftIntake::get, null);
    builder.addDoubleProperty("Intake Speed (right)", rightIntake::get, null);
  }

}
