// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public enum DriveBase{
        Comp2024(),PracticeBot();
    }

    public static final boolean demoMode = false;
    public static final DriveBase driveBase = DriveBase.Comp2024;

    public static final class Swerve {
        
        // Physical Constants
        public static final double driveBaseWidth = (driveBase.equals(DriveBase.PracticeBot)) ? 0.762 : 0.4445;
        public static final double driveBaseLength = (driveBase.equals(DriveBase.PracticeBot)) ? 0.762 : 0.4445; //meters
        public static final double driveGearRatio = 6.12; // L3
        public static final double steerGearRatio = 12.8; 
        public static final double wheelDiameterMeters = 0.1016;//0.098; // Measure and check later. Compensating for tread wear over comp could be cool
        public static final double driveConversionFactor = wheelDiameterMeters * Math.PI / driveGearRatio;

        public static final double maxSpeed = (demoMode ? 2.0 : 4.8); // m/s
        public static final double maxAngularSpeed = (demoMode ? 1.5 : 3.5); // rad/s
        public static final double driverThetaDeadband = 0.05;
        public static final double driverTranslationDeadband = 1;
        // Electrical Constants
        public static final int  driveMotorFreeCurrentLimit = 40; // TODO raise these and observe max speed differences
        public static final int  driveMotorStallCurrentLimit = 30;
        public static final int  steerMotorFreeCurrentLimit = 30;
        public static final int  pigeonId = 4;

        
        // Controller Gains
        public static final double kDriveP = 0.025;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;

        public static final double kSteerP = 0.004;//0.0025;
        public static final double kSteerI = 0.0;
        public static final double kSteerD = 0.0;

        public static final double kHeadingP = 0.04;
        public static final double kHeadingI = 0.0;
        public static final double kHeadingD = 0.0;
        public static final double kHeadingFF = 0.0;
        
        // Module Constants
        public enum ModulePosition{
            FR(0),BR(1),BL(2),FL(3);
            public final int modPos;
            
            private ModulePosition(int modPos){
                this.modPos = modPos;
            }

            public int getVal(){
                return this.modPos;
            }

            

        }
        public enum Module{
            /*       
            From Translation2d docs: 
            When the robot is placed on the origin, facing toward the X direction, 
            moving forward increases the X, whereas moving to the left increases the Y.

            Mod{modPos,driveMotorid,steerMotorid,cancoderid,displacment(x,y),drive kV, drive kS}
            */
            FR(ModulePosition.FR,
            (driveBase.equals(DriveBase.PracticeBot)) ? 10 : 20,
            (driveBase.equals(DriveBase.PracticeBot)) ? 11 : 21,
            (driveBase.equals(DriveBase.PracticeBot)) ? 12 : 22,
            (driveBase.equals(DriveBase.PracticeBot)) ? -7.029375 :180-29.61914,
            new Transform2d(new Translation2d(driveBaseLength/2,-driveBaseWidth/2),
            new Rotation2d()),
            (driveBase.equals(DriveBase.PracticeBot)) ? 2.443 : 2.4,
            (driveBase.equals(DriveBase.PracticeBot)) ?.25:0.0
            ), 
            BR(ModulePosition.BR,
            (driveBase.equals(DriveBase.PracticeBot)) ? 20 : 10,
            (driveBase.equals(DriveBase.PracticeBot)) ? 21 : 11,
            (driveBase.equals(DriveBase.PracticeBot)) ? 22 : 12,
            (driveBase.equals(DriveBase.PracticeBot)) ? 86.302734 :270-256.5,
            new Transform2d(new Translation2d(-driveBaseLength/2,-driveBaseWidth/2),
            new Rotation2d()),
            (driveBase.equals(DriveBase.PracticeBot)) ? 2.409: 2.4,
            (driveBase.equals(DriveBase.PracticeBot)) ? .15 : 0
            ),
            BL(ModulePosition.BL,
            (driveBase.equals(DriveBase.PracticeBot)) ? 30 : 40,
            (driveBase.equals(DriveBase.PracticeBot)) ? 31 : 41,
            (driveBase.equals(DriveBase.PracticeBot)) ? 32 : 42,
            (driveBase.equals(DriveBase.PracticeBot)) ? -103.614375 :-238.008+180 ,
            new Transform2d(new Translation2d(-driveBaseLength/2,driveBaseWidth/2),
            new Rotation2d()),
            (driveBase.equals(DriveBase.PracticeBot)) ? 2.413 : 2.4,
            (driveBase.equals(DriveBase.PracticeBot)) ? .25 : 0.0
            ),
            FL(ModulePosition.FL,
            (driveBase.equals(DriveBase.PracticeBot)) ? 40 : 30,
            (driveBase.equals(DriveBase.PracticeBot)) ? 41 : 31,
            (driveBase.equals(DriveBase.PracticeBot)) ? 42 : 32,
            (driveBase.equals(DriveBase.PracticeBot)) ? 112.154688 :-111.5332,
            new Transform2d(new Translation2d(driveBaseLength/2,driveBaseWidth/2),
            new Rotation2d()),
            (driveBase.equals(DriveBase.PracticeBot)) ? 2.387 : 2.4,
            (driveBase.equals(DriveBase.PracticeBot)) ?.23 : 0.0
            ); 
            
            public final ModulePosition modPos;
            public final int driveMotorid;
            public final int steerMotorid;
            public final int canCoderid;
            public final double canCoderOffset;
            public final Transform2d displacment; // from robot origin
            public final double kV;
            public final double kS;
            
            private Module(ModulePosition modPos, int driveMotorid, int steerMotorid, int canCoderid,double canCoderOffset, Transform2d displacment, double kV, double kS){
                this.modPos = modPos;
                this.driveMotorid = driveMotorid;
                this.steerMotorid = steerMotorid;
                this.canCoderid = canCoderid;
                this.canCoderOffset = canCoderOffset;
                this.displacment = displacment;
                this.kV = kV;
                this.kS = kS;
            }

        }
      
    }
}


