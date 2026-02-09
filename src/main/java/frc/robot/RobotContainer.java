// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DeployWrist;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveWithAim;
import frc.robot.commands.DriveWithRange;
import frc.robot.commands.Index;
import frc.robot.commands.MoveIntakeWrist;
import frc.robot.commands.RPMShoot;
import frc.robot.commands.SpinIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private Swerve swerveDrive = new Swerve();
  private XboxController driveController = new XboxController(0);
  private CommandXboxController operatorController = new CommandXboxController(1);
  private Intake intake = new Intake();
  private Shooter ballShooter = new Shooter();
  private Indexer indexer = new Indexer();


  public RobotContainer() {
    DataLogManager.start(); //logs everything in Network Tables
    DriverStation.startDataLog(DataLogManager.getLog());  //logs joystick values

    // find these in Elastic under '+Add Widget'
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData("PDH Readouts", new PowerDistribution());
    SmartDashboard.putData(intake);
    SmartDashboard.putData(ballShooter);
    SmartDashboard.putData(indexer);

    setDefaultCommands();
    configureBindings();
  }

  private void configureBindings() {

    //Driver commands
    new Trigger(()->driveController.getAButton()).whileTrue(new RunCommand(()->swerveDrive.setXMode(), swerveDrive));
    new Trigger(()->driveController.getRightBumperButton()).whileTrue(new DriveWithAim(
      ()-> driveController.getLeftY(), 
      ()->driveController.getLeftX(),
       swerveDrive));
    new Trigger(()->driveController.getLeftBumperButton()).whileTrue(new DriveWithRange(
      ()->driveController.getLeftX(),
       swerveDrive,
        VisionConstants.idealShootingRange));

    //Operator Commands
    operatorController.rightBumper().whileTrue(new Index(()->1, indexer));
    operatorController.x().onTrue(new InstantCommand(()->intake.setTargetSetpoint(SubsystemConstants.wristOut), intake));
    operatorController.a().onTrue(new DeployWrist(intake));
    operatorController.axisGreaterThan(2, 0.3).whileTrue(new SpinIntake(()->operatorController.getLeftTriggerAxis(), intake));
    operatorController.axisGreaterThan(3, 0.3).whileTrue(new RPMShoot(()->operatorController.getRightTriggerAxis(), ballShooter));
    operatorController.axisMagnitudeGreaterThan(1, 0.3).whileTrue(new MoveIntakeWrist(()->-operatorController.getLeftY(), intake));
    operatorController.button(8).onTrue(new InstantCommand(()->intake.zeroEncoders(), intake));
    
  }

  private void setDefaultCommands(){
    swerveDrive.setDefaultCommand(new Drive(
      ()->-driveController.getLeftY(),
      ()->-driveController.getLeftX(),
      ()->driveController.getRightX(),
      swerveDrive
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
