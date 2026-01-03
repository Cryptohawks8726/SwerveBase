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
public final class Constants {

    public static final boolean demoMode = false;

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

        public static final double joystickDeadband = 0; //TODO: UPDATE this if drivers want
    }
    
    public static final class OdometryConstants {
        // Tag offsets used with the driveToPose function of a Limelight.java instance
        // x axis is forward bot distance from tag
        // y axis is offset right (positive) or left
        // rotation is measured in radians
        //TODO: UPDATE WHEN GAME MANUAL RELEASES
        public static final Pose2d nearGoalAimingPosition = new Pose2d(2.6, 0, new Rotation2d(0)); // Rotation is 0
                                                                                                   // since the back of
                                                                                                   // the robot is doing
                                                                                                   // the shooting 

        //TODO: UPDATE DURING BOT FABRICATION
        // Camera position relative to robot center
        public static final double forwardOffset = 0.17145; // 6.75 in
        public static final double sideOffset = -0.00635; // 0.25 in
        public static final double upOffset = 0.36195; // 14.25 in
        public static final double rollOffset = 0;
        public static final double pitchOffset = 20; // Angled up or down
        public static final double yawOffset = 0; // Angled left or right

        public static final int innerGoalApriltag = DriverStation.getAlliance() //TODO: UPDATE ONCE GAME MANUAL RELEASES!!
                .orElseGet(() -> Alliance.Blue) == Alliance.Blue ? 5 : 6;
        public static final int outerGoalApriltag = DriverStation.getAlliance()
                .orElseGet(() -> Alliance.Blue) == Alliance.Blue ? 7 : 8;

        public static final int gyroID = 1;
    }
}
