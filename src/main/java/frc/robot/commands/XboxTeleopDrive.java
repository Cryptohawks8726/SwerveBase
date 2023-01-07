package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import java.lang.Math;

public class XboxTeleopDrive extends CommandBase{
    private final XboxController controller;
    
    private final SwerveDrive drivetrain;
    private Rotation2d lastHeading;
    private boolean isHeadingSet;
    private PIDController headingPID;
    
    public XboxTeleopDrive(SwerveDrive drivetrain,XboxController controller){
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
        boolean isRobotRelative = controller.getRawButtonPressed(XboxController.Button.kLeftBumper.value);
        
        // Get Controller Values
        double xVel = controller.getRawAxis(XboxController.Axis.kRightY.value); 
        double yVel = controller.getRawAxis(XboxController.Axis.kRightX.value);

        // Angular Velocity
        double thetaVel = controller.getRawAxis(XboxController.Axis.kLeftX.value) * Constants.Swerve.maxAngularSpeed;
        
        xVel = Math.signum(xVel) * Math.pow(xVel,2) * Constants.Swerve.maxSpeed; //square input while preserving sign
        yVel = Math.signum(yVel) * Math.pow(yVel,2) * Constants.Swerve.maxSpeed;
        
        // apply deadbands
        xVel = (Math.abs(xVel) < Constants.Swerve.driverTranslationDeadband) ? 0.0 : xVel;
        yVel = (Math.abs(yVel) < Constants.Swerve.driverTranslationDeadband) ? 0.0 : yVel;
        // maintain heading if there's no rotational input
        if (Math.abs(thetaVel) < Constants.Swerve.driverThetaDeadband){
            if (isHeadingSet == false){
                headingPID.reset();
                isHeadingSet = true;
                lastHeading = drivetrain.getPoseEstimate().getRotation();
                headingPID.setSetpoint(lastHeading.getDegrees()%360);
                thetaVel = headingPID.calculate(drivetrain.getPoseEstimate().getRotation().getDegrees()%360);
            }else{
                thetaVel = headingPID.calculate(drivetrain.getPoseEstimate().getRotation().getDegrees()%360);
            }
        } else{
            isHeadingSet = false;
        }

        drivetrain.drive(
            isRobotRelative ? new ChassisSpeeds(xVel, yVel, thetaVel)
            : ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, thetaVel,drivetrain.getPoseEstimate().getRotation())
            ,false
        );
    }

}
