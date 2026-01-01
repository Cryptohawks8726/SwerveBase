// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
//This text above me is boring should we change it?
public final class Constants {
    public static final boolean demoMode = false;
    //It would be funny if i deleted this
    public static final class SwerveConstants {
        public static final double maxSpeed = (demoMode ? 2.0 : 4.8); // m/s
        public static final double maxAngularSpeed = (demoMode ? 1.5 : 3); // rad/s

        public static final double defaultTranslationCoefficient = 1;
        public static final double defaultRotationCoefficient = 1;
        public static final double slowTranslationCoefficient = 0.1;
        public static final double slowRotationCoefficient = 0.1;

        public static final double positionkP = 0.5;
        public static final double positionkI = 0.0;
        public static final double positionkD = 0.0;

        public static final double rotationkP = 0.5;
        public static final double rotationkI = 0.0;
        public static final double rotationkD = 0.0;

        public static final double translationalErrorRange = 0.02;
        public static final double rotationalErrorRange = 0.1; // Radians
    }
//Not sure if we are supposed to keep this idk
//Hopefully not -_- this is why we did all the state stuff
    public static class OperatorConstants {
        // Joystick Deadband
        public static final double DEADBAND = 0.2;
        public static final double LEFT_Y_DEADBAND = 0.2;
        public static final double RIGHT_X_DEADBAND = 0.2;
        public static final double TURN_CONSTANT = 10;
    }
//Not sure if we are supposed to keep this idk
    public static final class AutoConstants {
        public static final Pose2d toTaxiLineFromInner1 = new Pose2d(0, 0, new Rotation2d(0));
        public static final Pose2d toTaxiLineFromInner2 = new Pose2d(0, 0, new Rotation2d(0));
        public static final Pose2d toTaxiLineFromOuter = new Pose2d(0, 0, new Rotation2d(0));

        public static final Pose2d pickUpLunites1 = new Pose2d(0, 0, new Rotation2d(0));
        public static final Pose2d pickUpLunites2 = new Pose2d(0, 0, new Rotation2d(0));
        public static final Pose2d pickUpLunites3 = new Pose2d(0, 0, new Rotation2d(0));
    }
//Not sure if we are supposed to keep this idk
    public static final class VisionConstants {
        // Tag offsets used with the driveToPose function of a Limelight.java instance
        // x axis is forward bot distance from tag
        // y axis is offset right (positive) or left
        // rotation is measured in radians
        public static final Pose2d nearGoalAimingPosition = new Pose2d(2.6, 0, new Rotation2d(0)); // Rotation is 0
                                                                                                   // since the back of
                                                                                                   // the robot is doing
                                                                                                   // the shooting
        // Camera position relative to robot center
        public static final double forwardOffset = 0.17145; // 6.75 in
        public static final double sideOffset = -0.00635; // 0.25 in
        public static final double upOffset = 0.36195; // 14.25 in
        public static final double rollOffset = 0;
        public static final double pitchOffset = 20; // Angled up or down
        public static final double yawOffset = 0; // Angled left or right

        public static final int innerGoalApriltag = DriverStation.getAlliance()
                .orElseGet(() -> Alliance.Blue) == Alliance.Blue ? 5 : 6;
        public static final int outerGoalApriltag = DriverStation.getAlliance()
                .orElseGet(() -> Alliance.Blue) == Alliance.Blue ? 7 : 8;

        // Key-value list for all the apriltags
        public static final LightweightAprilTagLayout layout = new LightweightAprilTagLayout(8)
                .addApriltag(1, new Pose2d(1.8288, 8.128, new Rotation2d(Math.PI * 1.5))) // Red Outer Starspire Station
                .addApriltag(2, new Pose2d(14.6304, 8.128, new Rotation2d(Math.PI * 1.5))) // Blue Outer Starspire
                                                                                           // Station
                .addApriltag(3, new Pose2d(0.1016, 6.858, new Rotation2d(0))) // Red Inner Starspire Station
                .addApriltag(4, new Pose2d(16.3576, 6.858, new Rotation2d(Math.PI))) // Blue Outer Starspire Station
                .addApriltag(5, new Pose2d(0.1016, 4.981575, new Rotation2d(0))) // Blue Inner Cosmic Converter
                .addApriltag(6, new Pose2d(16.3576, 4.981575, new Rotation2d(Math.PI))) // Red Inner Cosmic Converter
                .addApriltag(7, new Pose2d(0.1016, 0.5207, new Rotation2d(0))) // Blue Outer Cosmic Converte
                .addApriltag(8, new Pose2d(16.3576, 0.5207, new Rotation2d(Math.PI)));// Red Outer Cosmic Converter

        public static final int gyroID = 1;
    }
    //:)
    public static class Subsystem1Constants {
        public static final int MOTOR1_CAN_ID = 67;
        public static final int MOTOR2_CAN_ID = 67;
        public static final int MOTOR3_CAN_ID = 67;
    }

    public static class Subsystem2Constants {
        public static final int MOTOR1_CAN_ID = 67;
        public static final int MOTOR2_CAN_ID = 67;
        public static final int MOTOR3_CAN_ID = 55;
    }

    public static class Subsystem3Constants {
        public static final int MOTOR1_CAN_ID = 67;
        public static final int MOTOR2_CAN_ID = 67;
        public static final int MOTOR3_CAN_ID = 67;
    }
}
