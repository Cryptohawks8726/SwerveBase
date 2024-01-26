package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveDrive;

public class ActualXboxTeleopDrive extends CommandBase {
    private CommandXboxController controller;
    private SwerveDrive drivetrain;
    private double lastHeading;
    private boolean isHeadingSet;
    private PIDController headingPID;
    private double translationalSpeed,thetaSpeed;
    
    public ActualXboxTeleopDrive(SwerveDrive drivetrain, CommandXboxController controller){
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
        headingPID = new PIDController(-Swerve.kHeadingP, Swerve.kHeadingI, Swerve.kHeadingD, 20);
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

        boolean isRobotRelative = controller.leftTrigger().getAsBoolean();
        translationalSpeed = controller.rightTrigger().getAsBoolean() ? Swerve.maxSpeed : 2.5;
        thetaSpeed = controller.rightTrigger().getAsBoolean() ? Swerve.maxAngularSpeed : 2.0;
        // Get Controller Values
        double xVel = (Math.abs(controller.getLeftY()) > 0.15 ? controller.getLeftY() : 0.0); 
        double yVel = (Math.abs(controller.getLeftX()) > 0.15 ? controller.getLeftX() : 0.0);
        double thetaVel = (Math.abs(controller.getRightX()) > 0.20 ? -controller.getRightX() * thetaSpeed : 0.0);
        xVel = -Math.signum(xVel) * Math.pow(xVel,2) * translationalSpeed ;//square input while preserving sign
        yVel = -Math.signum(yVel) * Math.pow(yVel,2) * translationalSpeed ;

        // maintain heading if there's no rotational input
         if (thetaVel == 0.0 && ((Math.abs(xVel)>0.2) ||(Math.abs(yVel)>0.2))){
            if (isHeadingSet == false){
                headingPID.reset();
                isHeadingSet = true;
                lastHeading = drivetrain.gyro.getYaw()+180%360;
                headingPID.setSetpoint(lastHeading);
                thetaVel = headingPID.calculate(drivetrain.gyro.getYaw()+180%360);
             }else{
                thetaVel = headingPID.calculate(drivetrain.gyro.getYaw()+180%360);
            }
        } else{
            isHeadingSet = false;
        }
        
        drivetrain.drive(
             isRobotRelative ? new ChassisSpeeds(xVel, yVel, thetaVel)
            : ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, thetaVel, drivetrain.getRobotAngle())
            ,true
        );
    }

}
