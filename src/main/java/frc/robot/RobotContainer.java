// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Arm;
import frc.robot.commands.ActualXboxTeleopDrive;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {

  private final CommandXboxController driverController;
  //private final CommandXboxController operatorController;
  private final SwerveDrive drivetrain;
  //private final ShooterSubsystem shooter;
  //private final ClimberSubsystem climber;
  //private final ArmSubsystem arm;

  public RobotContainer() {

    Unmanaged.setPhoenixDiagnosticsStartTime(-1);
    drivetrain = new SwerveDrive();
    //shooter = new ShooterSubsystem();
    //climber = new ClimberSubsystem();
    //arm = new ArmSubsystem();
    
    driverController = new CommandXboxController(0);
    //operatorController = new CommandXboxController(1);

    configureBindings();
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(new ActualXboxTeleopDrive(drivetrain,driverController).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    driverController.start().onTrue(drivetrain.resetGyroAngle());
    /*operatorController.leftTrigger().onTrue(shooter.startIntake());
    operatorController.rightTrigger().onTrue(shooter.fireNote(false));//arm.atStatePos(Arm.ampState)
    operatorController.leftBumper().onTrue(shooter.nudgeIntake());

    operatorController.a().onTrue(arm.rotateToState(Arm.intakeState));
    operatorController.b().onTrue(arm.rotateToState(Arm.ampState));
    operatorController.y().onTrue(arm.rotateToState(Arm.sourceState));

    operatorController.back().onTrue(climber.smartReleaseClimber());
    operatorController.start().onTrue(climber.smartClimb());*/
    
  }

  public Command getAutonomousCommand() {
    if(!Constants.demoMode){

    PathPlannerPath choreoTraj = PathPlannerPath.fromChoreoTrajectory("2NoteAutoCenter"); //default path
    for (EventMarker event : choreoTraj.getEventMarkers()){
      
    }
    
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    if(Constants.path == Constants.autoPath.twoRingAutoMid){ //if robot is centered with the subwoofer
      drivetrain.setOdometryPosition(new Pose2d(1.3604378700256348, 5.526217937469482, new Rotation2d(3.1415)));
      choreoTraj = PathPlannerPath.fromChoreoTrajectory("2NoteAutoCenter");
       for (EventMarker event : choreoTraj.getEventMarkers()){
        event.getCommand().execute();
        /*if(event.getCommand().execute();){
          new InstantCommand(() -> {
            new WaitCommand(2);
          });
        }else if(event.getClass().getName().equals("Shoot first note")){
          new InstantCommand(() -> {
            new WaitCommand(3);
          });
        }else if(event.getClass().getName().equals("Shoot second note")){
          new InstantCommand(() -> {
            new WaitCommand(3);
          });
        }*/
      }
    }else if(Constants.path == Constants.autoPath.twoRingAutoRight){ //if the robot is angled on the ride side (facing the speaker) of the subwoofer. Top left of the robot's bumper aligned with the edge of the subwoofer
      drivetrain.setOdometryPosition(new Pose2d(0.7466598153114319, 6.684545040130615, new Rotation2d(-2.094395307179586)));
      choreoTraj = PathPlannerPath.fromChoreoTrajectory("2NoteAutoRight");
    }else if(Constants.path == Constants.autoPath.twoRingAutoLeft){
      drivetrain.setOdometryPosition(new Pose2d(0.7523629665374756, 4.439711570739746, new Rotation2d(2.0943951023931)));
      choreoTraj = PathPlannerPath.fromChoreoTrajectory("2NoteAutoLeft");
    }
    
    SmartDashboard.putNumber("Gyro angle:", drivetrain.getRobotAngle().getDegrees()%360);
    
    
    return AutoBuilder.followPath(choreoTraj);
    //returnq new InstantCommand(()->drivetrain.drive(new ChassisSpeeds(1, 0, 0.1), false),drivetrain)
    //.andThen(new WaitCommand(5))
    //.andThen(()->drivetrain.drive(new ChassisSpeeds(0, 0, 0), false),drivetrain);
  } else{ return null;}
  //return null;

}
}
