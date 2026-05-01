package frc.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.Constants.OdometryConstants;

public class PhotonOdometrySource extends OdometrySource {
    // Field in NetworkTables for visualizing the odometry.
    final Field2d field = new Field2d();
    final Matrix<N3, N1> stdDevs;
    final String name;
    final PhotonCamera camera;
    final PhotonPoseEstimator poseEstimator;
    final SwerveSubsystem swerve;

    /**
     * Creates a new PhotonOdometrySource with the given camera name, measurement
     * standard deviations, and a Transform3d object for the position of the camera
     * relative to the robot.
     * 
     * @param name          Camera's nickname set via PhotonVision configuration
     * @param stdDevs       Measurement standard deviations passed to YAGSL.
     *                      Standard devs can be made using VecBuilder.fill() and
     *                      there are 3 in the form
     *                      [x, y, rotation]
     * @param robotToCamera Transform from the robot to the camera's real position.
     */
    public PhotonOdometrySource(String name, Matrix<N3, N1> stdDevs, SwerveSubsystem swerve,
            Transform3d robotToCamera) {
        this.name = name;
        camera = new PhotonCamera(name);
        this.stdDevs = stdDevs;
        this.swerve = swerve;

        poseEstimator = new PhotonPoseEstimator(Constants.aprilTagLayout, robotToCamera);
        SmartDashboard.putData(name + "/Field Visualization", field);
    }

    @Override
    public Pose2d getCurrentPoseEstimate() throws VisionException {
        var results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            throw new VisionException("No new results.");
        } else {
            // we only handle one result currently

            var latestResult = results.get(results.size() - 1);
            if (Math.abs(swerve
                    .getFieldVelocity().omegaRadiansPerSecond) < OdometryConstants.ODO_TRUST_ROTATIONAL_SPEED_LIMIT) {
                var visionEst = poseEstimator.estimateCoprocMultiTagPose(latestResult);
                if (visionEst.isEmpty()) {
                    visionEst = poseEstimator.estimateLowestAmbiguityPose(latestResult);
                }

                if (visionEst.isEmpty()) {
                    throw new VisionException("Failed to get a valid pose estimate.");
                } else {
                    var pose = visionEst.get().estimatedPose.toPose2d();
                    field.setRobotPose(pose);
                    return pose;
                }
            } else {
                throw new VisionException("Rotational speed too high");
            }
        }
    }

    @Override
    public Matrix<N3, N1> getMeasurementStdDevs() {
        return stdDevs;
    }
}
