// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ActualXboxTeleopDrive;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {

  private final SwerveDrive drivetrain;
  private final CommandXboxController driverController;

  public RobotContainer() {

    Unmanaged.setPhoenixDiagnosticsStartTime(-1);
    drivetrain = new SwerveDrive();
    driverController = new CommandXboxController(0);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drivetrain.setDefaultCommand(new ActualXboxTeleopDrive(drivetrain,driverController).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    
    driverController.a().onTrue(drivetrain.resetGyroAngle());

    // Trigger driverRightBumper = driverController.rightBumper();
    // driverRightBumper.whileTrue(drivetrain.passiveBrake());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if(!Constants.demoMode){
    // An ExampleCommand will run in autonomou`s
    PathPlannerPath path = PathPlannerPath.fromPathFile("Test1");

    // Create a path following command using AutoBuilder. This will also trigger event markers.

    SmartDashboard.putNumber("e", 1);
    drivetrain.setOdometryPosition(new Pose2d(2.659489631652832, 7.026157379150391, new Rotation2d(0)));
    SmartDashboard.putNumber("Gyro angle:", drivetrain.getRobotAngle().getDegrees()%360);
    PathPlannerPath exampleChoreoTraj = PathPlannerPath.fromChoreoTrajectory("Path4");
    
    return AutoBuilder.followPath(exampleChoreoTraj);
    //return new InstantCommand(()->drivetrain.drive(new ChassisSpeeds(1, 0, 0.1), false),drivetrain)
    //.andThen(new WaitCommand(5))
    //.andThen(()->drivetrain.drive(new ChassisSpeeds(0, 0, 0), false),drivetrain);
  } else{ return null;}
}
}
