package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import java.lang.Math;

public class XboxTeleopDrive extends Command{
    // private final CommandXboxController controller;
    private final CommandJoystick controller;
    
    private final SwerveDrive drivetrain;
    private Rotation2d lastHeading;
    private boolean isHeadingSet;
    private PIDController headingPID;

    // TODO: Implement Optimization
    
    public XboxTeleopDrive(SwerveDrive drivetrain, /*CommandXboxController controller*/ CommandJoystick controller){
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
        headingPID = new PIDController(Constants.Swerve.kHeadingP, Constants.Swerve.kHeadingI, Constants.Swerve.kHeadingD, 20);
        headingPID.enableContinuousInput(0, 360);
    }
    @Override
    public void initialize(){
        isHeadingSet = false;
        headingPID.reset();
    } 
     
    @Override
    public void execute(){
        /* X axis is forward from driver perspective, and the Y axis is parallel to the driver station wall. 
    See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html 
        The controller axes have x as left-right and y as up-down
        */
        // boolean isRobotRelative = controller.leftBumper().getAsBoolean();

        boolean isRobotRelative = controller.trigger().getAsBoolean();
        isRobotRelative=false;
    
        // Get Controller Values
        // FIXME: xVel and yVel pull the wrong values? Flip them
        // double xVel = (Math.abs(controller.getLeftY()) > 0.1 ? controller.getLeftY() : 0.0); 
        // double yVel = (Math.abs(controller.getLeftX()) > 0.1 ? controller.getLeftX() : 0.0);
        // double thetaVel = (Math.abs(controller.getRightX()) > 0.1 ? controller.getRightX() * Constants.Swerve.maxAngularSpeed : 0.0);
        double xVel = controller.getY();
        double yVel = controller.getX();
        double thetaVel = controller.getZ();
         yVel = (Math.abs(xVel) > 0.18 ? Math.signum(xVel) * Math.pow(xVel,2) * Constants.Swerve.maxSpeed : 0.0); 
         xVel = (Math.abs(yVel) > 0.18 ? Math.signum(yVel) * Math.pow(yVel,2) * Constants.Swerve.maxSpeed : 0.0);
         thetaVel = (Math.abs(thetaVel) > 0.18 ? thetaVel * Constants.Swerve.maxAngularSpeed : 0.0);
        //thetaVel = 0.0;

       // xVel = Math.signum(xVel) * Math.pow(xVel,2) * Constants.Swerve.maxSpeed; //square input while preserving sign
       // yVel = Math.signum(yVel) * Math.pow(yVel,2) * Constants.Swerve.maxSpeed;

        // maintain heading if there's no rotational input
         /*  if (Math.abs(thetaVel) < 0.1){
            if (isHeadingSet == false){
                headingPID.reset();
                isHeadingSet = true;
                lastHeading = drivetrain.getRobotAngle();
                headingPID.setSetpoint(lastHeading.getDegrees()%180);
                thetaVel = headingPID.calculate(drivetrain.getRobotAngle().getDegrees()%180);
            }else{
                thetaVel = headingPID.calculate(drivetrain.getRobotAngle().getDegrees()%180);
            }
        } else{
            isHeadingSet = false;
        }*/
        SmartDashboard.putString("DriveOut", xVel + " " + yVel + " " + thetaVel);
        drivetrain.drive(
             isRobotRelative ? new ChassisSpeeds(xVel, yVel, thetaVel)
            : ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, thetaVel, drivetrain.getRobotAngle())
            ,true
        );
    }

}
