// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
    // CHANGE BEFORE MATCHES!!!!!
    public static final boolean isBlueAlliance = false;

    // Update per year. 67
    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final class SwerveConstants {
        public static final double maxSpeed = (demoMode ? 2.0 : 4.8); // m/s
        public static final double maxAngularSpeed = (demoMode ? 1.5 : 3); // rad/s

        public static final double defaultTranslationCoefficient = 1;
        public static final double defaultRotationCoefficient = 1;
        public static final double slowTranslationCoefficient = 0.3;
        public static final double slowRotationCoefficient = 0.3;

        public static final double positionkP = 0.5;
        public static final double positionkI = 0.0;
        public static final double positionkD = 0.0;

        public static final double rotationkP = 0.5;
        public static final double rotationkI = 0.0;
        public static final double rotationkD = 0.0;

        public static final double translationalErrorRange = 0.02;
        public static final double rotationalErrorRange = 0.1; // Radians

        public static final double joystickDeadband = 0.1; // TODO: UPDATE this if drivers want
    }

    public static final class OdometryConstants {
        // rad/s
        public static final double ODO_TRUST_ROTATIONAL_SPEED_LIMIT = 2 * Math.PI;

        // FOR LIMELIGHTs and PhotonVision Cams
        // x axis is forward bot distance from tag
        // y axis is offset right (positive) or left
        // rotation is measured in radians
        // distance is measured in meters
        // TODO: UPDATE DURING BOT FABRICATION
        // Camera position relative to robot center
        // LL Cam
        public static final double ll1forwardOffset = 0; // 0 in
        public static final double ll1sideOffset = 0; // 0 in
        public static final double ll1upOffset = 0; // 0 in
        public static final double ll1rollOffset = 0; // we will most likely never use roll
        public static final double ll1pitchOffset = 20; // Angled up or down
        public static final double ll1yawOffset = 0; // Angled left or right
        // PV Cam
        public static final Transform3d pv1Offset = new Transform3d(
                0, // 0 in (forward offset)
                0, // 0 in (right offset)
                0, // 0in (up offset)
                new Rotation3d(
                        0, // roll (most likely never used)
                        0, // pitch (up or down)
                        0 // yaw (left or right)
                ));

    }
}
