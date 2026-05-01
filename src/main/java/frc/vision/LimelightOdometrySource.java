package frc.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Constants.OdometryConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

/**
 * An OdometrySource which provides pose estimate from Limelight's MegaTag2
 * algorithm.
 */
public class LimelightOdometrySource extends OdometrySource {
    // Field in NetworkTables for visualizing the odometry.
    final Field2d field = new Field2d();
    // measurement standard deviations used by yagsl
    final Matrix<N3, N1> stdDevs;
    // name of the limelight.
    final String name;
    // Access point to the limelight's data dump
    final NetworkTable limelightLogs;
    // Needed to get gyro readings
    final SwerveSubsystem swerve;

    int lastTag = 0;

    /**
     * Constructs a new LimelightOdometrySource with given measurement standard
     * deviations
     * and a name.
     * Also requires the exact 3D position of the limelight relative to the robot,
     * with each
     * value provided separately as a double in meters or degrees :3
     * 
     * Standard devs can be made using VecBuilder.fill() and there are 3 in the form
     * [x, y, rotation]
     */
    public LimelightOdometrySource(Matrix<N3, N1> stdDevs, String limelightName, SwerveSubsystem swerve,
            double forwardOffset, double rightOffset, double upOffset, double rollOffset, double pitchOffset,
            double yawOffset) {
        name = limelightName;
        this.stdDevs = stdDevs;
        this.swerve = swerve;

        LimelightHelpers.setCameraPose_RobotSpace(name,
                forwardOffset,
                rightOffset,
                upOffset,
                rollOffset,
                pitchOffset,
                yawOffset);

        // Source of limelight data
        limelightLogs = NetworkTableInstance.getDefault().getTable(name);

        // Simulated field
        SmartDashboard.putData(name + "/Field Visualization", field);
    }

    /**
     * Same as the other constructor but supports a whitelist of tags to use for
     * pose estimation.
     */
    public LimelightOdometrySource(Matrix<N3, N1> stdDevs, String limelightName, SwerveSubsystem swerve,
            int[] allowedTagIDs,
            double forwardOffset, double rightOffset, double upOffset, double rollOffset, double pitchOffset,
            double yawOffset) {
        this(stdDevs, limelightName, swerve, forwardOffset, rightOffset, upOffset, rollOffset, pitchOffset, yawOffset);
        LimelightHelpers.SetFiducialIDFiltersOverride(name, allowedTagIDs);
    }

    @Override
    public Pose2d getCurrentPoseEstimate() throws VisionException {
        Pose2d limelightBotPose = new Pose2d(0, 0, new Rotation2d(0));

        // Adding robot heading to LimelightHelpers. Necessary for MegaTag2 (The best
        // apriltag odometry algorithm)
        // Called even when no tag is in sight since the swerve odometry (same pose as
        // in the one used in
        // swerve.getSwerveDrive().addVisionMeasurement(limelightBotPose, tsValue);)...
        // ...works even when not able to see a tag
        LimelightHelpers.SetRobotOrientation(
                name,
                swerve.getRawGyroPosition().getDegrees(),
                0, 0, 0, 0, 0);

        // If the camera does not see a tag, the tagID should be 0
        int tagID = getVisibleApriltag();

        // Adds limelight measurement to swerve odometry if the limelight is able to see
        // a tag
        if (tagID > 0) {
            if (Math.abs(swerve
                    .getFieldVelocity().omegaRadiansPerSecond) < OdometryConstants.ODO_TRUST_ROTATIONAL_SPEED_LIMIT) {
                // Getting MegaTag2 Pose Estimate
                PoseEstimate megaTagPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
                limelightBotPose = new Pose2d(megaTagPoseEstimate.pose.getX(), megaTagPoseEstimate.pose.getY(),
                        swerve.getRawGyroPosition());

                field.setRobotPose(limelightBotPose);

                // Sets the most recently seen tag
                lastTag = tagID;
            } else {
                throw new VisionException("Rotational speed too high");
            }
        } else {
            throw new VisionException("No apriltag");
        }

        // Will be 0 if no tag is in sight
        SmartDashboard.putNumber(name + "/Last Seen Tag", lastTag);

        return limelightBotPose;
    }

    @Override
    public Matrix<N3, N1> getMeasurementStdDevs() {
        return stdDevs;
    }

    public int getVisibleApriltag() {
        return (int) limelightLogs.getEntry("tid").getInteger(0);
    }

}
