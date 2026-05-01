package frc.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * An abstract class for cameras or other methods of determining the robot's
 * current pose.
 */
public abstract class OdometrySource {
    public class VisionException extends Exception {
        public VisionException(String message) {
            super(message);
        }

        public VisionException() {
            super("Unknown error");
        }
    }

    /**
     * Get the current pose estimated by the camera. This method may fail, so
     * make sure to wrap it in a try-catch block in case the camera cannot find
     * a pose estimate.
     * 
     * @return
     * @throws VisionException
     */
    public abstract Pose2d getCurrentPoseEstimate() throws VisionException;

    /**
     * Getter for the measurement standard deviations used for this camera.
     * 
     * @return Measurement standard deviations which should be passed to YAGSL's
     *         `SwerveDrive::addVisionMeasurement`
     */
    public abstract Matrix<N3, N1> getMeasurementStdDevs();
}
