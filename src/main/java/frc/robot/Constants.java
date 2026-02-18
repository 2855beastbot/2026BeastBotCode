// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


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

        public static final double intakeWristKp = 0.1;
        public static final double intakeWristKi = 0.0;
        public static final double intakeWristKd = 0.0;

        public static final double intakeWristKff = 1.33; // measured in volts
        public static final double wristOut = 0;
        public static final double wristIn = 1; // 2 is real value
        public static final double wristZeroVoltage = 33.0;

        public static final double wristGearboxCoef = (62.0/18)*(48.0/19);  
    }

    public static class VisionConstants{
        public static final String aimingLimelightName = "limelight-beast";
        public static final String locationLimelightName = "limelight-threea";

        public static final double idealShootingRange = 2;

        /**
         * 1 meter to number of rpm
         */
        //public static final double distanceToRPMRatio = 750;
        public static final double aimingTagHeight = 1.12395; //meter

        public static final double[] aimingConfig = {
            -0.2667, //forward (m) intake positive
            0.2413, //side (m) left positive
            0.2159, //up (m) 
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
}
