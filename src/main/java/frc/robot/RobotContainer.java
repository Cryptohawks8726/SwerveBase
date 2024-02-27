// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final CommandXboxController operatorController;
  private final SwerveDrive drivetrain;
  private final ShooterSubsystem shooter;
  private final ClimberSubsystem climber;
  private final ArmSubsystem arm;

  public RobotContainer() {

    Unmanaged.setPhoenixDiagnosticsStartTime(-1);
    drivetrain = new SwerveDrive();
    shooter = new ShooterSubsystem();
    climber = new ClimberSubsystem();
    arm = new ArmSubsystem();
    
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    configureBindings();
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(new ActualXboxTeleopDrive(drivetrain,driverController).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    driverController.start().onTrue(drivetrain.resetGyroAngle());
    operatorController.leftTrigger().onTrue(shooter.startIntake());
    operatorController.rightTrigger().onTrue(shooter.fireNote(false));//arm.atStatePos(Arm.ampState)
    operatorController.leftBumper().onTrue(shooter.nudgeIntake());

    operatorController.a().onTrue(arm.rotateToState(Arm.intakeState));
    operatorController.b().onTrue(arm.rotateToState(Arm.ampState));
    operatorController.y().onTrue(arm.rotateToState(Arm.sourceState));
    operatorController.x().onTrue(shooter.testCooking());

    operatorController.back().onTrue(climber.smartReleaseClimber());
    operatorController.start().onTrue(climber.smartClimb());
    
  }

  public Command getAutonomousCommand() {
    /*if(!Constants.demoMode){
    // An ExampleCommand will run in autonomous
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
  } else{ return null;}*/
  return null;

}
}
