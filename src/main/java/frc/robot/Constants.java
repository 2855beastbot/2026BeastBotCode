// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

/** Add your docs here. */
public class Constants {

    public static class SwerveConstants{
        public static final double maxDriveSpeed = 4.5;
        public static final double maxTurnSpeed = 4;

        public static final double slowModeVal = 0.25;

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
        public static final double maxShooterRPM = 5000;
        public static final double intakeWristKp = 0.1;
        public static final double intakeWristKi = 0.0;
        public static final double intakeWristKd = 0.0;
        public static final double wristOut = 0.0;
        public static final double wristIn = 30.0;

        public static final double wristGearboxCoef = (62.0/18)*(48.0/18);  //9.1851851852;
    }

    public static class VisionConstants{
        public static final String aimingLimelightName = "";
        public static final String locationLimelightName = "";

        public static final double[] aimingConfig = {
            0.0, //forward
            0.0, //side
            0.0, //up
            0.0, //roll
            0.0, //pitch
            0.0 //yaw
        };

        public static final double[] locationConfig = {
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        };

        public static final int[] targetingIDs = {
            22, 23
        };

        public static final int[] allIDs = {
            1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36
        };
    }
}
