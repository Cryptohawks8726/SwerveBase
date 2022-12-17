package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import java.lang.Math;

public class XboxTeleopDrive extends CommandBase{
    private final Joystick controller;
    
    private final SwerveDrive drivetrain;
    public XboxTeleopDrive(SwerveDrive drivetrain,Joystick controller){
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }

     
    @Override
    public void execute(){
        /* X axis is forward from driver perspective, and the Y axis is parallel to the driver station wall. 
    See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html 
        The controller axes have x as left-right and y as up-down
        */
        boolean isFieldRelative = controller.getRawButtonPressed(XboxController.Button.kLeftBumper.value);
        double xVel = controller.getRawAxis(XboxController.Axis.kRightY.value);
        double yVel = controller.getRawAxis(XboxController.Axis.kRightX.value);
        double thetaVel = controller.getRawAxis(XboxController.Axis.kLeftX.value) * Constants.Swerve.maxAngularSpeed;
        xVel = Math.signum(xVel) * Math.pow(xVel,2) * Constants.Swerve.maxSpeed; //square input while preserving sign
        yVel = Math.signum(yVel) * Math.pow(yVel,2) * Constants.Swerve.maxSpeed;
        drivetrain.drive(
            isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, thetaVel,drivetrain.getheadingorsmth)
            :new ChassisSpeeds(xVel, yVel, thetaVel)
        );
    }

}
