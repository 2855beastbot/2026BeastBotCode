// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.SubsystemConstants;

public class IntakeWrist extends SubsystemBase {
  /** Creates a new IntakeWrist. */
  private SparkMax leftWrist = new SparkMax(CANIDConstants.intakeArmLeft, MotorType.kBrushless);
  private SparkMax rightWrist = new SparkMax(CANIDConstants.intakeArmRight, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();
  private SparkAbsoluteEncoder encoder;
  private double targetSetpoint;
  private boolean isOpenLoop;
  private PIDController pidController = new PIDController(SubsystemConstants.intakeWristKp,
      SubsystemConstants.intakeWristKi, SubsystemConstants.intakeWristKd);

  public IntakeWrist() {
    encoder = rightWrist.getAbsoluteEncoder();
    config.absoluteEncoder.inverted(true);
    setTargetSetpoint(getPose());
    config.closedLoop.pid(SubsystemConstants.intakeWristKp, SubsystemConstants.intakeWristKi,
        SubsystemConstants.intakeWristKd);
    // config.closedLoop.feedForward.kCos(0);
    rightWrist.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    pidController.enableContinuousInput(0, Math.PI * 2);  //why are there two pid controllers?
    SmartDashboard.putData("Intake PID controller", pidController);

    // make left follow right, now everything sent to right, left will do
    // automatically
    leftWrist.configure(new SparkMaxConfig().follow(rightWrist, true), null, PersistMode.kNoPersistParameters);
  }
  
  public double getTargetSetpoint() {
    return targetSetpoint;
  }

  private double getPose() {
    return encoder.getPosition();
  }

  private void setTargetSetpoint(double setpoint) {
    isOpenLoop = false;
    targetSetpoint = setpoint;
  }

  private void moveWrist(double speed) {
    isOpenLoop = true;
    rightWrist.set(-speed);
  }

  private boolean isOpenLoop() {
    return isOpenLoop;
  }

  private boolean isAtSetpoint() {
    return rightWrist.getClosedLoopController().isAtSetpoint();
  }

  // public double getOutputCurrent() {
  // return rightWrist.getOutputCurrent();
  // }

  private void runPID() {
    double angle = encoder.getPosition();
    if (angle > 2.15) {
      angle = angle - (Math.PI * 2);
    }
    rightWrist.set(-pidController.calculate(angle, targetSetpoint));
  }

  public Trigger isAtIntakePosition() {
    return new Trigger(() -> encoder.getPosition() < 0.5);
  }

  public Command manual(DoubleSupplier speed) {
    return runEnd(() -> moveWrist(speed.getAsDouble()),
        () -> setTargetSetpoint(encoder.getPosition()))
        .withName("Manual Control");
  }

  public Command zeroEncoders() {
    return run(
        () -> {
          leftWrist.getEncoder().setPosition(0.0);
          rightWrist.getEncoder().setPosition(0.0);
        })
        .withName("Zero Encoders");
  }

  public Command goToPosition(double setpoint) {
    return run(
        () -> setTargetSetpoint(setpoint)).until(() -> isAtSetpoint())
        .withName("Go to position: " + setpoint);
  }

  public Command deploy() {
    return runEnd(
        () -> {
          moveWrist(1);
        },
        () -> {
          moveWrist(0);
          zeroEncoders();
          setTargetSetpoint(encoder.getPosition());
        })
        .until(() -> rightWrist.getOutputCurrent() > SubsystemConstants.wristZeroVoltage)
        .withName("Deploy");
  }

  public Command juggle() {
    return new SequentialCommandGroup(
        goToPosition(SubsystemConstants.wristMid), new WaitCommand(0.3),
        goToPosition(SubsystemConstants.wristOut), new WaitCommand(0.25),
        goToPosition(SubsystemConstants.wristIn), new WaitCommand(0.5),
        goToPosition(SubsystemConstants.wristOut), new WaitCommand(0.25))
        .withName("Juggle");
  }

  @Override
  public void periodic() {
    if (!isOpenLoop) {
      runPID();
    }

    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
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
  }

}
