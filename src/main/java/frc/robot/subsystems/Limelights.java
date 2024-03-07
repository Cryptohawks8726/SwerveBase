package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelights extends SubsystemBase {

    @Override
    public void periodic() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tid = table.getEntry("tid");

        double tagID = tid.getDouble(0.0);

        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0); // targetOffsetAngle_Vertical
        double area = ta.getDouble(0.0);

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
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("HorDist", horDistFromGoal);
        SmartDashboard.putNumber("DiagDist", diagDistFromGoal);
        SmartDashboard.putNumber("TagID", tagID);
    }
}


