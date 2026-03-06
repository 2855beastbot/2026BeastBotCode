// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DeployWrist;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveWithAim;
import frc.robot.commands.DriveWithRange;
import frc.robot.commands.Index;
import frc.robot.commands.MoveIntakeWrist;
import frc.robot.commands.RPMShoot;
import frc.robot.commands.ShootWithRange;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.WristJuggle;
import frc.robot.commands.autoCommands.AutoShoot;
import frc.robot.commands.autoCommands.ExtendHopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveInputStream;

public class RobotContainer {
  private Swerve swerveDrive = new Swerve();
  private XboxController driveController = new XboxController(0);
  private CommandXboxController operatorController = new CommandXboxController(1);
  private IntakeWrist intakeWrist = new IntakeWrist();
  private Intake intake = new Intake(intakeWrist);
  private Shooter ballShooter = new Shooter();
  private Indexer indexer = new Indexer();
  private LED LEDstrip = new LED();
  

  private SendableChooser<String> autoChooser = new SendableChooser<>();
  private String leftAuto = "Left";
  private String rightAuto = "Right";
  private String centerAuto = "center";
  private Pose2d targetHub;
   
  private RepeatCommand wristJuggle = new RepeatCommand(new SequentialCommandGroup(new WristJuggle(intakeWrist, SubsystemConstants.wristMid), new WristJuggle(intakeWrist, SubsystemConstants.wristIn)));   

  public RobotContainer() {
    
    autoChooser.addOption("Right auto", rightAuto);
    autoChooser.addOption("Left auto", leftAuto);
    autoChooser.addOption("Center auto", centerAuto);
    DataLogManager.start(); //logs everything in Network Tables
    DriverStation.startDataLog(DataLogManager.getLog());  //logs joystick values

    // find these in Elastic under '+Add Widget'
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData("PDH Readouts", new PowerDistribution());
    SmartDashboard.putData(intake);
    SmartDashboard.putData(ballShooter);
    SmartDashboard.putData(indexer);
    SmartDashboard.putData(swerveDrive);
    SmartDashboard.putData(swerveDrive.getAimingCamera());
    SmartDashboard.putData("auto selector", autoChooser);
    SmartDashboard.putData(intakeWrist);
    
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
        targetHub = (alliance.get() == Alliance.Blue) ? VisionConstants.blueHub : VisionConstants.redHub;
      }else{
        targetHub = VisionConstants.blueHub;
      }
    setDefaultCommands();
    configureBindings();
    LEDstrip.setPattern(LEDConstants.yellow);
    
      
  }

  private void configureBindings() {
    SwerveInputStream driveWithPose = SwerveInputStream.of(
    swerveDrive.getSwerve(),
     ()->-driveController.getLeftY(), 
     ()->-driveController.getLeftX())
     //.withControllerRotationAxis(()->driveController.getRightX())
     .deadband(0.3)
     .scaleTranslation(0.8)
     .aim(targetHub)
    .aimWhile(()->true);


    new Trigger(()->DriverStation.isFMSAttached()).onTrue(new InstantCommand(()->swerveDrive.updateAlliance(), swerveDrive).alongWith(new InstantCommand(()->setDefaultCommands())));
    new Trigger(()->DriverStation.isEnabled()).onTrue(new InstantCommand(()->swerveDrive.updateAlliance()).alongWith(new InstantCommand(()->setDefaultCommands())));

    //Driver commands
    new Trigger(()->driveController.getYButton()).whileTrue(new RunCommand(()->swerveDrive.setXMode(), swerveDrive));
    // new Trigger(()->driveController.getRightTriggerAxis() > 0.5).whileTrue(new ParallelCommandGroup(
    //   swerveDrive.driveWithInputStream(driveWithPose),
    //   new ShootWithRange(()->swerveDrive.getRPMFromRange(swerveDrive.getDistanceFromHub()), ballShooter)
    //   ));
        new Trigger(()->driveController.getRightTriggerAxis() > 0.5).whileTrue(new ParallelCommandGroup(
      new RunCommand(()->swerveDrive.drivePose(new Translation2d(-driveController.getLeftY(), -driveController.getLeftX()))),
      new ShootWithRange(()->swerveDrive.getRPMFromRange(swerveDrive.getDistanceFromHub()), ballShooter)
      ));
      
    /* 
    new Trigger(()->driveController.getLeftBumperButton()).whileTrue(new DriveWithRange(
      ()->-MathUtil.applyDeadband(driveController.getLeftX(), 0.1),
       swerveDrive,
        VisionConstants.idealShootingRange));
    */
    new Trigger(()->driveController.getRawButton(8)).onTrue(new InstantCommand(()->swerveDrive.resetOdometryWithAlliance(new Pose2d(swerveDrive.getPose2d().getX(), swerveDrive.getPose2d().getX(), new Rotation2d()))));
    new Trigger(()->driveController.getRawButton(7)).onTrue(new InstantCommand(()->swerveDrive.resetOdometryWithAlliance(swerveDrive.getAimingCamera().getPose())));
    new Trigger(()->driveController.getXButton()).onTrue(new InstantCommand(()->intakeWrist.setTargetSetpoint(SubsystemConstants.wristOut)).alongWith(new PrintCommand("intake out")));
    new Trigger(()->driveController.getAButton()).onTrue(new InstantCommand(()->intakeWrist.setTargetSetpoint(SubsystemConstants.wristMid)));
    new Trigger(()->driveController.getBButton()).onTrue(new InstantCommand(()->intakeWrist.setTargetSetpoint(SubsystemConstants.wristIn)));
    // new Trigger(()->driveController.getRightBumperButton()).whileTrue(new Index(()->1, indexer));
    new Trigger(()->driveController.getLeftTriggerAxis() > 0.3).whileTrue(new SpinIntake(()->driveController.getLeftTriggerAxis(), intake));
    new Trigger(()->driveController.getLeftBumperButton()).whileTrue(new SpinIntake(()->-1, intake));


    new Trigger(()->driveController.getRightBumperButton()).whileTrue(new ParallelCommandGroup(new Index(()->1, indexer), wristJuggle));

    //Operator Commands
    operatorController.rightBumper().whileTrue(new Index(()->1, indexer));
    operatorController.leftBumper().whileTrue(new SpinIntake(()->-1, intake));
    operatorController.x().onTrue(new InstantCommand(()->intakeWrist.setTargetSetpoint(SubsystemConstants.wristOut), intakeWrist));
    operatorController.b().onTrue(new InstantCommand(()->intakeWrist.setTargetSetpoint(SubsystemConstants.wristIn), intakeWrist));
    //operatorController.a().onTrue(new DeployWrist(intake));
    operatorController.axisGreaterThan(2, 0.3).whileTrue(new SpinIntake(()->operatorController.getLeftTriggerAxis(), intake));
    operatorController.axisGreaterThan(3, 0.3).whileTrue(new RunCommand(()->ballShooter.spin(operatorController.getRightTriggerAxis(), false), ballShooter));
    operatorController.axisMagnitudeGreaterThan(1, 0.3).whileTrue(new MoveIntakeWrist(()->-operatorController.getLeftY(), intakeWrist));
    operatorController.button(8).onTrue(new InstantCommand(()->intakeWrist.zeroEncoders(), intakeWrist));
    operatorController.y().whileTrue(new RunCommand(()->ballShooter.spin(5000, true), ballShooter));
    
  }

  private void setDefaultCommands(){
      var alliance = DriverStation.getAlliance();
       swerveDrive.setDefaultCommand(new Drive(
            ()->-MathUtil.applyDeadband(driveController.getLeftY(), 0.1),
            ()->-MathUtil.applyDeadband(driveController.getLeftX(), 0.1),
            ()->-driveController.getRightX(),
            swerveDrive));
            
      if(alliance.isPresent()){
        if(alliance.get() == Alliance.Blue) {
          swerveDrive.setDefaultCommand(new Drive(
            ()->-MathUtil.applyDeadband(driveController.getLeftY(), 0.1),
            ()->-MathUtil.applyDeadband(driveController.getLeftX(), 0.1),
            ()->-driveController.getRightX(),
            swerveDrive));
        } else {
          swerveDrive.setDefaultCommand(new Drive(
            ()->MathUtil.applyDeadband(driveController.getLeftY(), 0.1),
            ()->MathUtil.applyDeadband(driveController.getLeftX(), 0.1),
            ()->-driveController.getRightX(),
            swerveDrive));
        }
      }


    


  }

  public Command getAutonomousCommand() {
    NamedCommands.registerCommand("AutoShoot", new AutoShoot(ballShooter, swerveDrive, indexer));
    NamedCommands.registerCommand("HopperJuggle", wristJuggle);
    NamedCommands.registerCommand("ExtendHopper", new ExtendHopper(intake, intakeWrist));
    NamedCommands.registerCommand("StartWheels", new RunCommand(()->intake.spin(1), intake).asProxy());
    return new PathPlannerAuto(autoChooser.getSelected());
  }
}
