// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        public static final double driveBaseWidth = 2.0;
        public static final double driveBaseLength = 2.0;
        
        // Electrical Constants
        // TODO: add current limits
        
        // Controller Gains
        // TODO: add PID + FF constants
        
        // Module Constants
        public enum Module{
            /*       
            From Translation2d docs: 
            When the robot is placed on the origin, facing toward the X direction, 
            moving forward increases the X, whereas moving to the left increases the Y.

            Mod{modPos,driveMotorid,steerMotorid,cancoderid,displacment(x,y)}
            */
            NE(0,0,0,0,new Translation2d(driveBaseLength/2,driveBaseWidth/2)), 
            SE(1,0,0,0,new Translation2d(-driveBaseLength/2,driveBaseWidth/2)),
            SW(2,0,0,0,new Translation2d(-driveBaseLength/2,-driveBaseWidth/2)),
            NW(3,0,0,0,new Translation2d(driveBaseLength/2,-driveBaseWidth/2)); 
            
            public final int modPos;
            public final int driveMotorid;
            public final int steerMotorid;
            public final int cancoderid;
            public final Translation2d displacment; // from robot origin
            
            private Module(int modPos, int driveMotorid, int steerMotorid, int cancoderid, Translation2d displacment){
                this.modPos = modPos;
                this.driveMotorid = driveMotorid;
                this.steerMotorid = steerMotorid;
                this.cancoderid = cancoderid;
                this.displacment = displacment;
            }

        }
      
    }
}
