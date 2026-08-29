// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.VisionConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX left = new TalonFX(CANIDConstants.shooterLeft);
  private TalonFX right = new TalonFX(CANIDConstants.shooterRight);
  private final double passiveTargetRPM = SubsystemConstants.maxShooterRPM / 30;
  private double targetRPS; // making this RPS instead of RPM for better internal consistency, everything
                            // outside the class is still RPM
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private final DutyCycleOut leftDutyCycle;
  private final DutyCycleOut rightDutyCycle;
  private final VelocityVoltage leftVelocity;
  private final VelocityVoltage rightVelocity;

  public Shooter() {
    leftVelocity = new VelocityVoltage(passiveTargetRPM);
    rightVelocity = new VelocityVoltage(passiveTargetRPM);
    leftDutyCycle = new DutyCycleOut(0);
    rightDutyCycle = new DutyCycleOut(0);

    config.CurrentLimits.SupplyCurrentLowerLimit = 35;
    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.Slot0.kP = 0.04;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    left.getConfigurator().apply(config);
    right.getConfigurator().apply(config);

    setDefaultCommand(stop());
  }

  public Command shootDutycycle(DoubleSupplier speed) {
    return run(
        () -> {
          left.setControl(leftDutyCycle.withOutput(speed.getAsDouble()));
          right.setControl(rightDutyCycle.withOutput(speed.getAsDouble()));
        })
        .withName("Shoot Duty Cycle");
  }

  public Command shootDutycycle(double speed) {
    return shootDutycycle(() -> speed);
  }

  public Command shootRPM(DoubleSupplier rpm) {
    return run(
        () -> {
          double targetRPS = rpm.getAsDouble() / 60;
          left.setControl(leftVelocity.withVelocity(targetRPS));
          right.setControl(rightVelocity.withVelocity(targetRPS));
        })
        .withName("Shoot RPM");
  }

  public Command shootRPM(double rpm) {
    return shootRPM(() -> rpm);
  }

  public Command shootDistance(DoubleSupplier meters) {
    return shootRPM(() -> (VisionConstants.distanceToRPMRatio * meters.getAsDouble()) + VisionConstants.baseRPM)
        .withName("Shooting for Distance");
  }

  public Command stop() {
    return shootDutycycle(0).withName("Stopped");
  }

  @Override
  public void periodic() {
  }

  // TODO: add sysid functions for calibrating shooter speed

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // open Elastic -> Add Widget -> scroll to Shooter and open the dropdown -> drag
    // values onto dashboard
    builder.addDoubleProperty("Target RPS", () -> targetRPS, null);
    builder.addDoubleProperty("Left/Speed", left::get, null);
    builder.addDoubleProperty("Left/Current (A)", () -> left.getSupplyCurrent().getValueAsDouble(), null);
    builder.addDoubleProperty("Left/Temperature (C)", () -> left.getDeviceTemp().getValueAsDouble(), null);
    builder.addDoubleProperty("Right/Speed", right::get, null);
    builder.addDoubleProperty("Right/RPS", () -> right.getVelocity().getValueAsDouble(), null);
    builder.addDoubleProperty("Right/Current (A)", () -> right.getSupplyCurrent().getValueAsDouble(), null);
    builder.addDoubleProperty("Right/Temperature (C)", () -> right.getDeviceTemp().getValueAsDouble(), null);
  }
}
