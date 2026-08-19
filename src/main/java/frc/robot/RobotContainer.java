// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.WristJuggle;
import frc.robot.commands.autoCommands.AutoShoot;
import frc.robot.commands.autoCommands.ExtendHopper;
import frc.robot.simutils.RobotSim;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private RobotSim robotSim;
  
  private CommandXboxController driveController = new CommandXboxController(0);
  private CommandXboxController operatorController = new CommandXboxController(1);

  private Swerve swerveDrive = new Swerve();
  private IntakeWrist intakeWrist = new IntakeWrist();
  private Intake intake = new Intake();
  private Shooter ballShooter = new Shooter();
  private Indexer indexer = new Indexer();
  private LED LEDstrip = new LED();

  private Superstructure superstructure = new Superstructure(this);

  private SendableChooser<String> autoChooser = new SendableChooser<>();
  private String leftAuto = "Left";
  private String rightAuto = "Right";
  private String centerAuto = "center";
   
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

    NamedCommands.registerCommand("AutoShoot", new AutoShoot(ballShooter, swerveDrive, indexer));
    NamedCommands.registerCommand("HopperJuggle", wristJuggle);
    NamedCommands.registerCommand("ExtendHopper", new ExtendHopper(intakeWrist));
    NamedCommands.registerCommand("StartWheels", new RunCommand(()->intake.spin(1), intake).asProxy());
    
    setDefaultCommands();
    configureBindings();
    LEDstrip.setPattern(LEDConstants.yellow);
    
    if(RobotBase.isSimulation()){
      robotSim = new RobotSim();
    }
  }

  private void setDefaultCommands(){
    // contains all driving logic and state handling for teleop, it will get overridden by pathplanner's
    // path follow commands during auto
    swerveDrive.setDefaultCommand(swerveDrive.run(() -> 
        swerveDrive.teleopDrive(driveController.getLeftY(), 
        driveController.getLeftX(), driveController.getRightX())));

    superstructure.setDefaultCommand(superstructure.idleState());

    // if(AllianceInfo.isBlue()){
    //   swerveDrive.setDefaultCommand(new Drive(
    //         ()->-MathUtil.applyDeadband(driveController.getLeftY(), 0.1),
    //         ()->-MathUtil.applyDeadband(driveController.getLeftX(), 0.1),
    //         ()->-driveController.getRightX(),
    //         swerveDrive));
    // } else {
    //   swerveDrive.setDefaultCommand(new Drive(
    //         ()->MathUtil.applyDeadband(driveController.getLeftY(), 0.1),
    //         ()->MathUtil.applyDeadband(driveController.getLeftX(), 0.1),
    //         ()->-driveController.getRightX(),
    //         swerveDrive));
    // }
  }

  private void configureBindings() {
    driveController.a()
      .whileTrue(superstructure.pickingUpState());

    driveController.x()
      .whileTrue(superstructure.shootingState());
    
    // SwerveInputStream driveWithPose = SwerveInputStream.of(
    // swerveDrive.getSwerve(),
    //  ()->-driveController.getLeftY(), 
    //  ()->-driveController.getLeftX())
    //  //.withControllerRotationAxis(()->driveController.getRightX())
    //  .deadband(0.3)
    //  .scaleTranslation(0.8)
    //  .aim(targetHub)
    // .aimWhile(()->true);

    //Driver commands
    // new Trigger(()->driveController.getYButton()).whileTrue(new RunCommand(()->swerveDrive.setXMode(), swerveDrive));
    // new Trigger(()->driveController.getRightTriggerAxis() > 0.5).whileTrue(new ParallelCommandGroup(
    //   swerveDrive.driveWithInputStream(driveWithPose),
    //   new ShootWithRange(()->swerveDrive.getRPMFromRange(swerveDrive.getDistanceFromHub()), ballShooter)
    //   ));
      //   new Trigger(()->driveController.getRightTriggerAxis() > 0.5).whileTrue(new ParallelCommandGroup(
      // new RunCommand(()->swerveDrive.drivePose(new Translation2d(-driveController.getLeftY(), -driveController.getLeftX()))),
      // new ShootWithRange(()->swerveDrive.getRPMFromRange(swerveDrive.getDistanceFromHub()), ballShooter)
      // ));
      
    /* 
    new Trigger(()->driveController.getLeftBumperButton()).whileTrue(new DriveWithRange(
      ()->-MathUtil.applyDeadband(driveController.getLeftX(), 0.1),
       swerveDrive,
        VisionConstants.idealShootingRange));
    */
    // new Trigger(()->driveController.getRawButton(8)).onTrue(new InstantCommand(()->swerveDrive.resetOdometryWithAlliance(new Pose2d(swerveDrive.getPose2d().getX(), swerveDrive.getPose2d().getX(), new Rotation2d()))));
    // new Trigger(()->driveController.getRawButton(7)).onTrue(new InstantCommand(()->swerveDrive.resetOdometryWithAlliance(swerveDrive.getAimingCamera().getPose())));
    // new Trigger(()->driveController.getXButton()).onTrue(new InstantCommand(()->intakeWrist.setTargetSetpoint(SubsystemConstants.wristOut)).alongWith(new PrintCommand("intake out")));
    // new Trigger(()->driveController.getAButton()).onTrue(new InstantCommand(()->intakeWrist.setTargetSetpoint(SubsystemConstants.wristMid)));
    // new Trigger(()->driveController.getBButton()).onTrue(new InstantCommand(()->intakeWrist.setTargetSetpoint(SubsystemConstants.wristIn)));
    // // new Trigger(()->driveController.getRightBumperButton()).whileTrue(new Index(()->1, indexer));
    // new Trigger(()->driveController.getLeftTriggerAxis() > 0.3).whileTrue(new SpinIntake(()->driveController.getLeftTriggerAxis(), intake));
    // new Trigger(()->driveController.getLeftBumperButton()).whileTrue(new SpinIntake(()->-1, intake));


    // new Trigger(()->driveController.getRightBumperButton()).whileTrue(new ParallelCommandGroup(new Index(()->1, indexer), wristJuggle));

    //Operator Commands
    // operatorController.rightBumper().whileTrue(new Index(()->1, indexer));
    // operatorController.leftBumper().whileTrue(new SpinIntake(()->-1, intake));
    // operatorController.x().onTrue(new InstantCommand(()->intakeWrist.setTargetSetpoint(SubsystemConstants.wristOut), intakeWrist));
    // operatorController.b().onTrue(new InstantCommand(()->intakeWrist.setTargetSetpoint(SubsystemConstants.wristIn), intakeWrist));
    // //operatorController.a().onTrue(new DeployWrist(intake));
    // operatorController.axisGreaterThan(2, 0.3).whileTrue(new SpinIntake(()->operatorController.getLeftTriggerAxis(), intake));
    // operatorController.axisGreaterThan(3, 0.3).whileTrue(new RunCommand(()->ballShooter.spin(operatorController.getRightTriggerAxis(), false), ballShooter));
    // operatorController.axisMagnitudeGreaterThan(1, 0.3).whileTrue(new MoveIntakeWrist(()->-operatorController.getLeftY(), intakeWrist));
    // operatorController.button(8).onTrue(new InstantCommand(()->intakeWrist.zeroEncoders(), intakeWrist));
    // operatorController.y().whileTrue(new RunCommand(()->ballShooter.spin(5000, true), ballShooter));
    
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto(autoChooser.getSelected());
  }

  public Swerve getSwerve(){
    return swerveDrive;
  }

  public IntakeWrist getIntakeWrist(){
    return intakeWrist;
  }

  public Intake getIntake(){
    return intake;
  }

  public Shooter getShooter(){
    return ballShooter;
  }

  public Indexer getIndexer(){
    return indexer;
  }

  public LED getLEDStrip(){
    return LEDstrip;
  }
}
