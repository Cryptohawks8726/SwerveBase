package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import java.lang.Math;

public class XboxTeleopDrive extends CommandBase{
    private final CommandXboxController controller;
    
    private final SwerveDrive drivetrain;
    private Rotation2d lastHeading;
    private boolean isHeadingSet;
    private PIDController headingPID;

    // TODO: Implement Optimization
    
    public XboxTeleopDrive(SwerveDrive drivetrain,CommandXboxController controller){
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
         boolean isRobotRelative = controller.leftBumper().getAsBoolean();
        //boolean isRobotRelative = true;
        
        // Get Controller Values
        // FIXME: xVel and yVel pull the wrong values? Flip them
        double xVel = controller.getLeftY(); 
        double yVel = controller.getLeftX();
        double thetaVel = controller.getRightX();
        //thetaVel = 0;

         // apply deadbands
         xVel = (Math.abs(xVel) < Constants.Swerve.driverTranslationDeadband) ? 0.0 : xVel;
         yVel = (Math.abs(yVel) < Constants.Swerve.driverTranslationDeadband) ? 0.0 : yVel;
         thetaVel = (Math.abs(thetaVel) < Constants.Swerve.driverTranslationDeadband) ? 0.0 : thetaVel* Constants.Swerve.maxAngularSpeed; 
        // Angular Velocity
        
        
        xVel = Math.signum(xVel) * Math.pow(xVel,2) * Constants.Swerve.maxSpeed; //square input while preserving sign
        yVel = Math.signum(yVel) * Math.pow(yVel,2) * Constants.Swerve.maxSpeed;
        
       
        System.out.print("xVel");
        System.out.println(xVel);
        System.out.print("yVel");
        System.out.println(yVel);
        System.out.print("Theta");
        System.out.println(thetaVel);

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
            ,true
        );

        //drivetrain.drive(new ChassisSpeeds(xVel, yVel, thetaVel), false);
    }

}
