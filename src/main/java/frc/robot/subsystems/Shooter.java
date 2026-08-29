// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
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
  private double targetRPS; // making this RPS instead of RPM for better internal consistency, everything
                            // outside the class is still RPM
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private final DutyCycleOut leftDutyCycle;
  private final DutyCycleOut rightDutyCycle;
  private final VelocityVoltage leftVelocity;
  private final VelocityVoltage rightVelocity;

  public Shooter() {
    leftVelocity = new VelocityVoltage(0);
    rightVelocity = new VelocityVoltage(0);
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

  /**
   * <p>Determines shooting speed as a proportion (0.0 to 1.0) of max possible shooting speed</p>
   * <p>Intended for use with controller axes such as triggers</p>
   * @param speed
   * @return
   */
  public Command shootProportionalRPM(DoubleSupplier speed) {
    return run(
        () -> {
          double targetRPS = SubsystemConstants.maxShooterRPM * MathUtil.clamp(speed.getAsDouble(), 0, 1) / 60;
          left.setControl(leftVelocity.withVelocity(targetRPS));
          right.setControl(rightVelocity.withVelocity(targetRPS));
        })
        .withName("Shoot Proportional");
  }

  public Command shootProportionalRPM(double speed) {
    return shootProportionalRPM(() -> speed);
  }

  /**
   * Shoots at the exact rpm specified
   * @param rpm
   * @return
   */
  public Command shootExactRPM(DoubleSupplier rpm) {
    return run(
        () -> {
          double targetRPS = rpm.getAsDouble() / 60;
          left.setControl(leftVelocity.withVelocity(targetRPS));
          right.setControl(rightVelocity.withVelocity(targetRPS));
        })
        .withName("Shoot RPM");
  }

  public Command shootExactRPM(double rpm) {
    return shootExactRPM(() -> rpm);
  }

  public Command shootDistance(DoubleSupplier meters) {
    return shootExactRPM(() -> (VisionConstants.distanceToRPMRatio * meters.getAsDouble()) + VisionConstants.baseRPM)
        .withName("Shoot for Distance");
  }

  public Command stop() {
    return run(() -> {
      left.setControl(leftDutyCycle);
      right.setControl(rightDutyCycle);
    }).withName("Stopped");
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
