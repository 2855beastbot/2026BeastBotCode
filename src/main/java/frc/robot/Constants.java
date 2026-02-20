// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;


/** Add your docs here. */
public class Constants {

    public static class SwerveConstants{
        public static final double maxDriveSpeed = 4.5;
        public static final double maxTurnSpeed = 4;

        public static final double slowModeVal = 1;

        public static final PPHolonomicDriveController autoController = new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0));
        /* 
        public static final HolonomicPathFollowerConfig autoBuilderPathConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(8, 0.0 ,0), //original p = 5, 1st attempt: p = 5, d = 0.5, 2nd attempt: p= 5, d = 0.5, 3rd attempt: p = 5, d = 3 this caused the wheels to shutter
            new PIDConstants(5, 0.0, 0), //5.0, 0, 0.2
            maxDriveSpeed, // Max module speed, in m/s
            11.5, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig());
            */
    }

    public static class CANIDConstants{
        public static final int intakeLeft = 20;
        public static final int intakeRight = 21;
        public static final int intakeArmLeft = 22;
        public static final int intakeArmRight = 23;
        public static final int indexer = 31;
        public static final int shooterLeft = 32;
        public static final int shooterRight = 33;
    }

    public static class SubsystemConstants{
        public static final double maxShooterRPM = 6000;

        public static final double intakeWristKp = 0.4;
        public static final double intakeWristKi = 0.0;
        public static final double intakeWristKd = 0.0;

        public static final double intakeWristKff = 1.33; // measured in volts
        public static final double wristOut = 0.2;
        public static final double wristIn = 2.15; 
        public static final double wristZeroVoltage = 33.0;

        public static final double wristGearboxCoef = (62.0/18)*(48.0/19);  
    }

    public static class VisionConstants{
        public static final String aimingLimelightName = "limelight-beast";
        public static final String locationLimelightName = "limelight-threea";

        public static final double idealShootingRange = 2;

        public static final Pose2d blueHub = new Pose2d(4.02844, 4.11861, new Rotation2d());
        public static final Pose2d redHub = new Pose2d(12.51204, 4.11861, new Rotation2d());

        /**
         * 1 meter to number of rpm
         */
        public static final double distanceToRPMRatio = 650;
        public static final double baseRPM = 3425;
        public static final double aimingTagHeight = 1.12395; //meter

        public static final double[] aimingConfig = {
            -0.26, //forward (m) intake positive
            0.24, //side (m) left positive
            0.185, //up (m) 
            0.0, //roll (deg) around Y axis
            30.0, //pitch (deg) around X axis
            180 //yaw (deg) around Z axis
        };

        public static final double[] locationConfig = {
            0.0, //forward
            0.0, //side
            0.0, //up
            0.0, //roll
            0.0, // pitch
            0.0 //yaw
        };

        public static final int[] targetingIDs = {
            24,25,26,27, //blue tags
            8,9,10,11 //red tags
        };

        public static final int[] allIDs = {
            1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32
        };
    }

    public static class LEDConstants{
        public static final LEDPattern green = LEDPattern.solid(Color.kGreen);
        public static final LEDPattern red = LEDPattern.solid(Color.kRed);
        public static final LEDPattern yellow = LEDPattern.solid(Color.kYellow);
        public static final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
        public static final Distance ledSpacing = Meters.of(1/120.0);
        public static final LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing);
        public static final LEDPattern breatheYellow = yellow.breathe(Seconds.of(2));
    }
}
