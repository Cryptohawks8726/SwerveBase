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
    
    public static final class Swerve {
        
        // Physical Constants
        public static final double driveBaseWidth = 0.762;
        public static final double driveBaseLength = 0.762; //meters
        public static final double driveGearRatio = 6.12; // L3
        public static final double steerGearRatio = 12.8; 
        public static final double wheelDiameterMeters = 0.098; // Measure and check later. Compensating for tread wear over comp could be cool
        public static final double driveConversionFactor = wheelDiameterMeters * Math.PI / driveGearRatio;

        public static final double maxSpeed = 7.0; // m/s, I have no clue if this is realistic // TODO testing
        public static final double maxAngularSpeed = 9.0; // rad/s
        public static final double driverThetaDeadband = 0.05;
        public static final double driverTranslationDeadband = 1;
        // Electrical Constants
        public static final int  driveMotorFreeCurrentLimit = 30;
        public static final int  driveMotorStallCurrentLimit = 30;
        public static final int  driveMotorfreeCurrentLimit = 30;


        
        // Controller Gains
        // TODO: Tune PID + FF constants
        public static final double kDriveP = 0.025;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveFF = 0.0;

        public static final double kSteerP = 0.002;
        public static final double kSteerI = 0.0;
        public static final double kSteerD = 0.0;
        public static final double kSteerFF = 0.0;

        public static final double kHeadingP = 0.0;
        public static final double kHeadingI = 0.0;
        public static final double kHeadingD = 0.0;
        public static final double kHeadingFF = 0.0;
        
        // Module Constants
        public enum Module{
            /*       
            From Translation2d docs: 
            When the robot is placed on the origin, facing toward the X direction, 
            moving forward increases the X, whereas moving to the left increases the Y.

            Mod{modPos,driveMotorid,steerMotorid,cancoderid,displacment(x,y)}
            */

            //TODO: Define Forward
            NE(0,10,11,12,126.826,new Transform2d(new Translation2d(driveBaseLength/2,driveBaseWidth/2),new Rotation2d())), 
            SE(1,20,21,22,43.418,new Transform2d(new Translation2d(-driveBaseLength/2,driveBaseWidth/2),new Rotation2d())),
            SW(2,30,31,32,-47.549,new Transform2d(new Translation2d(-driveBaseLength/2,-driveBaseWidth/2),new Rotation2d())),
            NW(3,40,41,42,228.340,new Transform2d(new Translation2d(driveBaseLength/2,-driveBaseWidth/2),new Rotation2d())); 
            
            public final int modPos;
            public final int driveMotorid;
            public final int steerMotorid;
            public final int canCoderid;
            public final double canCoderOffset;
            public final Transform2d displacment; // from robot origin
            
            private Module(int modPos, int driveMotorid, int steerMotorid, int canCoderid,double canCoderOffset, Transform2d displacment){
                this.modPos = modPos;
                this.driveMotorid = driveMotorid;
                this.steerMotorid = steerMotorid;
                this.canCoderid = canCoderid;
                this.canCoderOffset = canCoderOffset;
                this.displacment = displacment;
            }

        }
      
    }
}
