// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.XboxTeleopDrive;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive drivetrain;
  private final CommandXboxController driverController;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    
    drivetrain = new SwerveDrive();
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
    drivetrain.setDefaultCommand(new XboxTeleopDrive(drivetrain,driverController).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    Trigger driverRightBumper = driverController.rightBumper();
    driverRightBumper.whileTrue(drivetrain.passiveBrake());
    Trigger driverRightTrigger = driverController.rightTrigger();
    driverRightTrigger.whileTrue(new RepeatCommand(new InstantCommand(()->drivetrain.normalZeroModules(),drivetrain)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public  Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    
    

    //config
    TrajectoryConfig config = new TrajectoryConfig(Constants.Swerve.kMaxSpeedMetersPerSecond, 
    Constants.Swerve.kMaxAccelerationMetersPerSecondSquared)
    .setKinematics(Constants.Swerve.kDriveKinematics);


    //actual path to follow
    Trajectory path1 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(0)), 
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),  
      new Pose2d(3, 0, new Rotation2d(0)), config);


    //tracking path

    PIDController xController = new PIDController(Constants.Swerve.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.Swerve.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.Swerve.kPThetaController, 0, 0, Constants.Swerve.kThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);


    //command to follow path
    SwerveControllerCommand swervepath = new SwerveControllerCommand(
      path1, 
      drivetrain::getPoseEstimate,
      Constants.Swerve.kDriveKinematics,
      xController,
      yController,
      thetaController,
      drivetrain::setModuleStates,
      drivetrain);
      
    return new SequentialCommandGroup(
       new InstantCommand(()->drivetrain.resetOdometry(path1.getInitialPose())),
      swervepath
     // return new RepeatCommand(swervepath);
    );

    
    
  } 
}
