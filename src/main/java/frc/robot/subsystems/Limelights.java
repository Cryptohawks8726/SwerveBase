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

        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0); // targetOffsetAngle_Vertical
        double area = ta.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0; 

        // distance from the center of the Limelight lens to the floor (inches)
        double limelightLensHeightInches = 20.0; 

        // distance from the target to the floor (inches) maybe convert to meters?
        double goalHeightInches = 60.0; 

        double angleToGoalDegrees = limelightMountAngleDegrees + y;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        //calculate horizontal distance from limelight to goal
        double horDistFromGoal = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("HorDist", horDistFromGoal);
    }
}


