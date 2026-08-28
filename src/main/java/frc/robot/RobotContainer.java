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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.Index;
import frc.robot.commands.ShootWithRange;
import frc.robot.commands.autoCommands.AutoShoot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private Swerve swerveDrive = new Swerve();
  private CommandXboxController driveController = new CommandXboxController(0);
  private CommandXboxController operatorController = new CommandXboxController(1);
  private IntakeWrist intakeWrist = new IntakeWrist();
  private Intake intake = new Intake();
  private Shooter ballShooter = new Shooter();
  private Indexer indexer = new Indexer();
  private LED LEDstrip = new LED();
  

  private SendableChooser<String> autoChooser = new SendableChooser<>();
  private String leftAuto = "Left";
  private String rightAuto = "Right";
  private String centerAuto = "center";

  private Pose2d targetHub;
  
   
  // private SequentialCommandGroup wristJuggle = new SequentialCommandGroup(
  //   new WristJuggle(intakeWrist, SubsystemConstants.wristMid), new WaitCommand(0.3),
  //   new WristJuggle(intakeWrist, SubsystemConstants.wristOut), new WaitCommand(0.25),
  //   new WristJuggle(intakeWrist, SubsystemConstants.wristIn), new WaitCommand(0.5), 
  //   new WristJuggle(intakeWrist, SubsystemConstants.wristOut), new WaitCommand(0.25));
  

  
  public RobotContainer() {
    
    autoChooser.addOption("Right auto", rightAuto);
    autoChooser.addOption("Right center auto", "Right to center");
    autoChooser.addOption("Right far auto", "Right to far");

    autoChooser.addOption("Left auto", leftAuto);
    autoChooser.addOption("Left center auto", "Left to center");
    autoChooser.addOption("Left far auto", "Left to far");

    autoChooser.addOption("Center auto", centerAuto);
    autoChooser.addOption("Race Center Right", "RaceCenterRight");
    autoChooser.addOption("Race Center Left", "RaceCenterLeft");
    //autoChooser.addOption("LeftTest", "LeftTest");
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

    NamedCommands.registerCommand("AutoShoot", new AutoShoot(ballShooter, swerveDrive, indexer));
    NamedCommands.registerCommand("HopperJuggle", intakeWrist.juggle());
    NamedCommands.registerCommand("ExtendHopper", intakeWrist.goToPosition(SubsystemConstants.wristOut).asProxy());
    NamedCommands.registerCommand("StartWheels", new RepeatCommand(intake.intakeSafely(() -> 1, intakeWrist.isAtIntakePosition())).withTimeout(0.1).asProxy());
    
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
    // SwerveInputStream driveWithPose = SwerveInputStream.of(
    // swerveDrive.getSwerve(),
    //  ()->-driveController.getLeftY(), 
    //  ()->-driveController.getLeftX())
    //  //.withControllerRotationAxis(()->driveController.getRightX())
    //  .deadband(0.3)
    //  .scaleTranslation(0.8)
    //  .aim(targetHub)
    // .aimWhile(()->true);


    new Trigger(()->DriverStation.isFMSAttached()).onTrue(new InstantCommand(()->swerveDrive.updateAlliance(), swerveDrive).alongWith(new InstantCommand(()->setDefaultCommands())));
    new Trigger(()->DriverStation.isEnabled()).onTrue(new InstantCommand(()->swerveDrive.updateAlliance()).alongWith(new InstantCommand(()->setDefaultCommands())));

    //Driver commands
    driveController.y().whileTrue(new RunCommand(()->swerveDrive.setXMode(), swerveDrive));
    // new Trigger(()->driveController.getRightTriggerAxis() > 0.5).whileTrue(new ParallelCommandGroup(
    //   swerveDrive.driveWithInputStream(driveWithPose),
    //   new ShootWithRange(()->swerveDrive.getRPMFromRange(swerveDrive.getDistanceFromHub()), ballShooter)
    //   ));
    new Trigger(()->driveController.getRightTriggerAxis() > 0.5).whileTrue(new ParallelCommandGroup(
      new RunCommand(()->swerveDrive.drivePose(new Translation2d(
        -MathUtil.applyDeadband(driveController.getLeftY(), 0.1),
        -MathUtil.applyDeadband(driveController.getLeftX(), 0.1)),
        targetHub)),
      new ShootWithRange(()->swerveDrive.getRPMFromRange(swerveDrive.getDistanceFromHub()), ballShooter)
      ));

    driveController.leftBumper().whileTrue(
      new ParallelCommandGroup(
        new RunCommand(()->swerveDrive.drivePose(
        new Translation2d(
          -MathUtil.applyDeadband(driveController.getLeftY(), 0.1), 
          -MathUtil.applyDeadband(driveController.getLeftX(), 0.1)),
        swerveDrive.determineFeedPose()),
        swerveDrive),
        new ShootWithRange(()->swerveDrive.getRPMFromRange(swerveDrive.getDistanceFromPose(swerveDrive.determineFeedPose())), ballShooter)
        ));

    //new Trigger(()->driveController.getPOV(0) == 180).onTrue(new InstantCommand(()->swerveDrive.setMaxDriveSpeedMult(1)));
    //new Trigger(()->driveController.getPOV(0) == 180).onFalse(new InstantCommand(()->swerveDrive.setMaxDriveSpeedMult(0.8)));
      
    /* 
    new Trigger(()->driveController.getLeftBumperButton()).whileTrue(new DriveWithRange(
      ()->-MathUtil.applyDeadband(driveController.getLeftX(), 0.1),
       swerveDrive,
        VisionConstants.idealShootingRange));
    */
    
    driveController.button(8).onTrue(new InstantCommand(()->swerveDrive.resetOdometryWithAlliance(new Pose2d(swerveDrive.getPose2d().getX(), swerveDrive.getPose2d().getX(), new Rotation2d()))));
    driveController.button(7).onTrue(new InstantCommand(()->swerveDrive.resetOdometryWithAlliance(swerveDrive.getAimingCamera().getPose())));
    driveController.x().onTrue(intakeWrist.goToPosition(SubsystemConstants.wristOut));
    driveController.a().onTrue(intakeWrist.goToPosition(SubsystemConstants.wristMid));
    driveController.b().onTrue(intakeWrist.goToPosition(SubsystemConstants.wristIn));
    // new Trigger(()->driveController.getRightBumperButton()).whileTrue(new Index(()->1, indexer));
    driveController.axisGreaterThan(2, 0.3).whileTrue(intake.intakeSafely(()->driveController.getLeftTriggerAxis(), intakeWrist.isAtIntakePosition()));
    driveController.povUp().whileTrue(intake.intakeSafely(() -> -1, intakeWrist.isAtIntakePosition()));



    driveController.rightBumper().whileTrue(new ParallelCommandGroup(
      new SequentialCommandGroup(
        new WaitCommand(0.1),
        new Index(()->1, indexer)),
      new SequentialCommandGroup(
        new WaitCommand(0.25),
        intakeWrist.juggle().repeatedly()
      )));

    //Operator Commands
    operatorController.rightBumper().whileTrue(new Index(()->1, indexer));
    operatorController.leftBumper().whileTrue(intake.intakeSafely(()->-1, intakeWrist.isAtIntakePosition()));
    operatorController.x().onTrue(intakeWrist.goToPosition(SubsystemConstants.wristOut));
    operatorController.b().onTrue(intakeWrist.goToPosition(SubsystemConstants.wristIn));
    //operatorController.a().onTrue(new DeployWrist(intake));
    operatorController.axisGreaterThan(2, 0.3).whileTrue(intake.intakeSafely(()->operatorController.getLeftTriggerAxis(), intakeWrist.isAtIntakePosition()));
    operatorController.axisGreaterThan(3, 0.3).whileTrue(new RunCommand(()->ballShooter.spin(operatorController.getRightTriggerAxis(), false), ballShooter));
    operatorController.axisMagnitudeGreaterThan(1, 0.3).whileTrue(intakeWrist.manual(()->-operatorController.getLeftY()));
    operatorController.button(8).onTrue(intakeWrist.zeroEncoders());
    operatorController.y().whileTrue(new RunCommand(()->ballShooter.spin(5000, true), ballShooter));
    operatorController.a().whileTrue(new RunCommand(()->ballShooter.spin(1000, true), ballShooter));
    
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
   
    return new PathPlannerAuto(autoChooser.getSelected());
  }
}
