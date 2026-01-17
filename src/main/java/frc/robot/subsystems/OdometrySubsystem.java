package frc.robot.subsystems;

import frc.robot.util.LimelightHelpers;
import frc.robot.util.Constants.OdometryConstants;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OdometrySubsystem extends SubsystemBase {
    // TODO: UPDATE kP and kD when testing bot
    private final Pose2d[] scoringSetpoints = new Pose2d[] {
            OdometryConstants.nearGoalAimingPosition,
    };
    private int debugSetpointIndex = 0;

    // State boolean to describe when the limelight is being used to autonomously
    // drive
    private boolean trackingTag = false;

    // State boolean to describe when the vision system is in its debug mode
    private boolean visionDebugMode = false;

    // Represents the ID of the most recently viewed allied reef tag. Used for
    // tag-relative reef allignment
    private int lastTag = 0;

    private final SwerveSubsystem swerve;

    // Access point to the limelight's data dump
    private final NetworkTable limelightLogs;

    // Simpler apriltag layout for 3rd-party games (i.e. Blair Bunnybots)
    private final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeAndyMark); // TODO: UPDATE ONCE MANUAL DROPS!!

    private final Field2d field = new Field2d();

    private final Pigeon2 johnGyro = new Pigeon2(OdometryConstants.gyroID);

    public OdometrySubsystem(SwerveSubsystem newSwerve) {
        LimelightHelpers.setCameraPose_RobotSpace("limelight-april",
                OdometryConstants.forwardOffset,
                OdometryConstants.sideOffset,
                OdometryConstants.upOffset,
                OdometryConstants.rollOffset,
                OdometryConstants.pitchOffset,
                OdometryConstants.yawOffset);

        // TODO: UDPATE if this has use cases during the season
        // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-april", ...);

        swerve = newSwerve;

        // Source of limelight data
        limelightLogs = NetworkTableInstance.getDefault().getTable("limelight-april");

        // Simulated field
        SmartDashboard.putData("Apriltag Bot Pose", field);

        // n1 = x, n2 = y, n3 = theta; each value represents the expected error of each
        // measurement; x and y are in meters, theta is in radians
        // While the limelight itself does not supply an angle, addvisionmeasurement
        // injects the measurement of the gyro to circumvent weird YAGSL stuff
        swerve.getSwerveDrive().setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 0.05));

        // TODO: UPDATE this if need be
        johnGyro.setYaw(DriverStation.getAlliance().orElseGet(() -> Alliance.Blue) == Alliance.Blue ? 180 : 0);
    }

    @Override
    public void periodic() {
        Pose2d limelightBotPose = new Pose2d(0, 0, new Rotation2d(0));

        // Adding robot heading to LimelightHelpers. Necessary for MegaTag2 (The best
        // apriltag odometry algorithm)
        // Called even when no tag is in sight since the swerve odometry (same pose as
        // in the one used in
        // swerve.getSwerveDrive().addVisionMeasurement(limelightBotPose, tsValue);)...
        // ...works even when not able to see a tag
        LimelightHelpers.SetRobotOrientation(
                "limelight-april",
                johnGyro.getYaw().getValueAsDouble(),
                0, 0, 0, 0, 0);

        // If the camera does not see a tag, the tagID should be 0
        int tagID = getVisibleApriltag();

        // Adds limelight measurement to swerve odometry if the limelight is able to see
        // a tag
        if (tagID > 0) {
            // Getting MegaTag2 Pose Estimate
            PoseEstimate megaTagPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-april");
            Pose2d inputPose = new Pose2d(megaTagPoseEstimate.pose.getX(), megaTagPoseEstimate.pose.getY(),
                    johnGyro.getRotation2d());

            // Adding vision measurement to the swerve pose
            // Note that the gyro angle is being added through the limelight pose in order
            // to circumvent YAGSL weirdness
            swerve.getSwerveDrive().addVisionMeasurement(inputPose, Timer.getFPGATimestamp());

            // Adding said bot pose to the simulated field
            field.setRobotPose(swerve.getPose());

            // Sets the most recently seen tag
            lastTag = tagID;
        }

        // Will be 0 if no tag is in sight
        SmartDashboard.putNumber("Apriltag Seen Tag ID", lastTag);

        // Debugging logs; will be 0 if no tag is in sight
        SmartDashboard.putNumberArray("Apriltag Odometry Pose", new double[] {
                limelightBotPose.getX(),
                limelightBotPose.getY(),
                limelightBotPose.getRotation().getRadians(),
        });
    }

    /**
     * Returns the currently visible apriltag as seen by the apriltag camera.
     * Returns 0 if no tag is visible.
     */
    public int getVisibleApriltag() {
        return (int) limelightLogs.getEntry("tid").getInteger(0);
    }

    /**
     * Gets state variable for whether or not a position setpoint is being tracked
     * 
     * @return True if a setpoint is being tracked, false if not
     */
    public boolean getIsTracking() {
        return trackingTag;
    }

    /**
     * Checks if the robot's vision system is in debug mode
     * 
     * @return True if the vision system is in debug mode
     */
    public boolean getIsInDebugMode() {
        return visionDebugMode;
    }

    /**
     * Enables and disables debug mode
     */
    public void toggleDebugMode() {
        visionDebugMode = !visionDebugMode;

        if (!visionDebugMode) {
            SmartDashboard.putString("DEBUG MODE SETPOINT", "DEBUG DISABLED");
            SmartDashboard.putNumberArray("DEBUG SETPT POS", new double[] { 0, 0, 0 });
        }
    }

    /**
     * Moves to debugging the next setpoint in the setpoint array
     */
    public void cycleDebugSetpoint() {
        debugSetpointIndex++;
        if (debugSetpointIndex >= scoringSetpoints.length)
            debugSetpointIndex = 0;

        String setpointOuput = "";

        switch (debugSetpointIndex) {
            case 0:
                setpointOuput = "nearGoalAimingPosition";
                break;
        }

        Pose2d debugPose = scoringSetpoints[debugSetpointIndex];

        SmartDashboard.putString("DEBUG MODE SETPOINT", setpointOuput);
        SmartDashboard.putNumberArray("DEBUG SETPT POS", new double[] {
                debugPose.getX(),
                debugPose.getY(),
                debugPose.getRotation().getDegrees()
        });
    }

    /**
     * Translate the robot's current debug setpoint (in meters)
     * 
     * @param changeX Move closer or farther from the front of the tag
     * @param changeY Move towards the left or right of the tag
     */
    public void translateDebugSetpoint(double changeX, double changeY) {
        Pose2d oldPose = scoringSetpoints[debugSetpointIndex];

        scoringSetpoints[debugSetpointIndex] = new Pose2d(
                oldPose.getX() + changeX,
                oldPose.getY() + changeY,
                oldPose.getRotation());

        Pose2d debugPose = scoringSetpoints[debugSetpointIndex];

        SmartDashboard.putNumberArray("DEBUG SETPT POS", new double[] {
                debugPose.getX(),
                debugPose.getY(),
                debugPose.getRotation().getDegrees()
        });

        // Imprinted in match logs
        System.out.println("UPDATE TO SETPOINT " + debugSetpointIndex + ": " +
                debugPose.getX() + ", " +
                debugPose.getY() + ", " +
                debugPose.getRotation() + ", ");
    }

    /**
     * Rotate the robot's current debug setpoint
     * 
     * @param changeTheta The rotation to edit the current setpoint by (in degrees)
     */
    public void rotateDebugSetpoint(double changeTheta) {
        Pose2d oldPose = scoringSetpoints[debugSetpointIndex];

        scoringSetpoints[debugSetpointIndex] = new Pose2d(
                oldPose.getX(),
                oldPose.getY(),
                Rotation2d.fromDegrees(oldPose.getRotation().getDegrees() + changeTheta));

        Pose2d debugPose = scoringSetpoints[debugSetpointIndex];

        SmartDashboard.putNumberArray("DEBUG SETPT POS", new double[] {
                debugPose.getX(),
                debugPose.getY(),
                debugPose.getRotation().getDegrees()
        });

        // Imprinted in match logs
        System.out.println("UPDATE TO SETPOINT " + debugSetpointIndex + ": " +
                debugPose.getX() + ", " +
                debugPose.getY() + ", " +
                debugPose.getRotation() + ", ");
    }

    /**
     * Gets the scoring setpoint that is currently being debugged if in debug mode,
     * returns null otherwise
     * 
     * @return The current scoring setpoint as a Pose2d object
     */
    public Optional<Pose2d> getCurrentDebugPose() {
        return Optional.ofNullable(visionDebugMode ? scoringSetpoints[debugSetpointIndex] : null);
    }

    public Pose2d getTagRelativePose(Pose2d desiredPose, int tagID) {
        if (!aprilTagLayout.getTagPose(tagID).isPresent())
            return new Pose2d(0, 0, new Rotation2d());

        // Uses the most recent tag for creating setpoints
        Pose2d tagPose = aprilTagLayout.getTagPose(tagID).get().toPose2d();

        // Creates a desired pose for the robot by converting the desired distance from
        // the tag in tag-relative space to the desired bot pose in field space.
        return tagPose.transformBy(
                new Transform2d(desiredPose.getX(), desiredPose.getY(), desiredPose.getRotation()));
    }

    /**
     * Plots a robot at the desired setpoint. Can be used to compare where the
     * robot's PID has it end up vs where it should be
     * 
     * @param desiredSetpoint
     */
    public void plotSimulatedSetpoint(Pose2d desiredSetpoint) {
        FieldObject2d positionSetpoint = field.getObject("currentSetpoint");

        positionSetpoint.setPose(desiredSetpoint);
    }

    /**
     * Plots a tag-relative robot setpoint on the Apriltag Bot Pose field2d. Purely
     * for debugging purposes
     * 
     * @param desiredDistance The desired pose of the robot relative to the given
     *                        apriltag
     * @param tagID           The ID of the apriltag which desiredDistance is
     *                        relative to
     */
    @SuppressWarnings("unused")
    public void simulateTagRelativeSetpoint(Pose2d desiredDistance, int tagID) {
        Pose2d tagPose = aprilTagLayout.getTagPose(tagID).get().toPose2d();

        Pose2d targetPose = tagPose.transformBy(
                new Transform2d(desiredDistance.getX(), desiredDistance.getY(), desiredDistance.getRotation()));

        FieldObject2d positionSetpoint = field.getObject("positionSetpoint" + tagID);

        positionSetpoint.setPose(targetPose);
    }
}