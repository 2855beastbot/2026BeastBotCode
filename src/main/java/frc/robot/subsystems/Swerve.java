// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */
  private SwerveDrive swerveDrive;
  public Swerve() {
    double maximumSpeed = Units.feetToMeters(4.5);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    try{
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }catch(IOException e){
      throw new RuntimeException(e);
    }
  }

  public double getMaxDriveSpeed(){
    return SwerveConstants.maxDriveSpeed;
  }

  public double getMaxTurnSpeed(){
    return SwerveConstants.maxTurnSpeed;
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
    swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
