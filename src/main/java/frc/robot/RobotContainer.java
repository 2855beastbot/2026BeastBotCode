// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANIDConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.RPMShoot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private Swerve swerveDrive = new Swerve();
  private XboxController driveController = new XboxController(0);
  private CommandXboxController operatorController = new CommandXboxController(1);
  private Intake intake = new Intake();

  private Shooter ballShooter = new Shooter();
  public RobotContainer() {
    setDefaultCommands();
    configureBindings();
  }

  private void configureBindings() {
    operatorController.axisGreaterThan(0, 0.3).whileTrue(new RPMShoot(()->operatorController.getRightTriggerAxis(), ballShooter));
    new Trigger(()->driveController.getAButton()).onTrue(new RunCommand(()->intake.runMotor(CANIDConstants.intakeArmRight), intake));
  }

  private void setDefaultCommands(){
    swerveDrive.setDefaultCommand(new Drive(
      ()->-driveController.getLeftY(),
      ()->-driveController.getLeftX(),
      ()->driveController.getRightX(),
      0.25,
      swerveDrive
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
