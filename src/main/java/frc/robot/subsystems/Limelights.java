package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Limelights extends SubsystemBase {
    NetworkTable table;
    
    public Limelights() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
        
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tid = table.getEntry("tid");

        double tagID = tid.getDouble(0.0);

        //read values periodically
        double x = tx.getDouble(0.0); // THESE ARE ANGLES
        double y = ty.getDouble(0.0); // targetOffsetAngle_Vertical
        double area = ta.getDouble(0.0); // the area of the apriltag on screen

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 30.0; 

        // distance from the center of the Limelight lens to the floor (inches)
        double limelightLensHeightInches = 26.0; 

        // distance from the target to the floor (inches) maybe convert to meters?
        double goalHeightInches = 52.0; 
        double dy = goalHeightInches-limelightLensHeightInches; // change in height from limelight to apriltag

        double angleToGoalDegrees = limelightMountAngleDegrees + y;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        //calculate horizontal distance from limelight to goal
        double horDistFromGoal = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        // calculate diagonal
        double diagDistFromGoal = Math.sqrt(dy*dy+Math.pow(horDistFromGoal, 2));

        //post to smart dashboard periodically
        SmartDashboard.putNumber("XAngleToApT", x);
        SmartDashboard.putNumber("YAngleToApT", y);
        //SmartDashboard.putNumber("HorDist", horDistFromGoal);
        //SmartDashboard.putNumber("DiagDist", diagDistFromGoal);
        SmartDashboard.putNumber("TagID", tagID);
        
        addVisionMeasurementToPoseEstimator();
    }

    public void addVisionMeasurementToPoseEstimator() { // TODO: add argument SwerveDrivePoseEstimator poseEstimator
        double[] botposeArray = table.getEntry("botpose").getDoubleArray(new double[6]);
        
        double xMeters = botposeArray[0]; // X translation (meters)
        double yMeters = botposeArray[1]; // Y translation (meters)
        //double zMeters = botposeArray[2]; // Z translation (meters)
        //double rollDegrees = botposeArray[3]; // Roll (degrees)
        //double pitchDegrees = botposeArray[4]; // Pitch (degrees)
        double yawDegrees = botposeArray[5]; // Yaw (degrees)

        // Create the Pose2d using the extracted values
        Pose2d robotPose = new Pose2d(xMeters, yMeters, new Rotation2d(yawDegrees));
        SmartDashboard.putNumber("x translation", xMeters);
        SmartDashboard.putNumber("y translation", yMeters);
        SmartDashboard.putNumber("poseYaw", yawDegrees);

        //poseEstimator.addVisionMeasurement(robotPose, Timer.getFPGATimestamp());
    }
}


