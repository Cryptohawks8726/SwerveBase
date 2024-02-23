// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ActualXboxTeleopDrive;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Com
 * mand-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  final SwerveDrive drivetrain;

  // private final CommandXboxController driverController;
  
  private final CommandXboxController driverController;

  private DoubleLogEntry log;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    Unmanaged.setPhoenixDiagnosticsStartTime(-1);
    drivetrain = new SwerveDrive();
    // driverController = new CommandXboxController(0);
    
    driverController = new CommandXboxController(0);

    DataLog logger = DataLogManager.getLog();
    log = new DoubleLogEntry(logger, "allLogs");

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
    
    driverController.a().onTrue(drivetrain.resetGyroAngle());

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

    drivetrain.setOdometryPosition(new Pose2d(3.3835394382476807, 7.026157379150391, new Rotation2d(0))); //the starting position of the robot
    SmartDashboard.putNumber("Gyro angle:", drivetrain.getRobotAngle().getDegrees()%360);
    PathPlannerPath exampleChoreoTraj = PathPlannerPath.fromChoreoTrajectory("Path4");
  
    
    return AutoBuilder.followPath(exampleChoreoTraj);

    



    //return new InstantCommand(()->drivetrain.drive(new ChassisSpeeds(1, 0, 0.1), false),drivetrain)
    //.andThen(new WaitCommand(5))
    //.andThen(()->drivetrain.drive(new ChassisSpeeds(0, 0, 0), false),drivetrain);
  } else{ return null;}
}
}
