// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.unmanaged.UnmanagedJNI;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ActualXboxTeleopDrive;
import frc.robot.commands.XboxTeleopDrive;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SwerveModule;

/**
 * This class is where the bulk of the robot should be declared. Since Com
 * mand-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive drivetrain;

  // private final CommandXboxController driverController;
  
  private final CommandXboxController driverController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    Unmanaged.setPhoenixDiagnosticsStartTime(-1);
    drivetrain = new SwerveDrive();
    // driverController = new CommandXboxController(0);
    
    driverController = new CommandXboxController(0);

    

    // Configure the button bindings
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
    
    
    /*Trigger sideButton = driverController.button(2); // the "side button" on the joystick decreases voltage
    sideButton.True(new InstantCommand(()->{
      drivetrain.modules.get(0).noVoltage();
      SmartDashboard.putNumber("FRVoltage", drivetrain.modules.get(0).wantedVoltage);
      ;}));*/

    // Trigger driverRightBumper = driverController.rightBumper();
    // driverRightBumper.whileTrue(drivetrain.passiveBrake());
    // Trigger driverRightTrigger = driverController.rightTrigger();
    // driverRightTrigger.whileTrue(new RepeatCommand(new InstantCommand(()->drivetrain.normalZeroModules(),drivetrain)));
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

    drivetrain.setOdometryPosition(new Pose2d(2.8405020236968994,7.009701728820801, new Rotation2d(0)));
    PathPlannerPath exampleChoreoTraj = PathPlannerPath.fromChoreoTrajectory("NewPath");
    return AutoBuilder.followPath(exampleChoreoTraj);
    //return new InstantCommand(()->drivetrain.drive(new ChassisSpeeds(1, 0, 0.1), false),drivetrain)
    //.andThen(new WaitCommand(5))
    //.andThen(()->drivetrain.drive(new ChassisSpeeds(0, 0, 0), false),drivetrain);
  } else{ return null;}
}
}
