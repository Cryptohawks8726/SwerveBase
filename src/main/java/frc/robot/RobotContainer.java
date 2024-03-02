// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Objects;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
    private final SendableChooser<Command> autoChooser;
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
      
      NamedCommands.registerCommand("ShootFirstNote", new WaitCommand(2.0));
      NamedCommands.registerCommand("IntakeNote", new WaitCommand(2.0));
      NamedCommands.registerCommand("ShootSecondNote", new WaitCommand(2.0));

      autoChooser = AutoBuilder.buildAutoChooser();

      configureBindings();

      SmartDashboard.putData("Auto Chooser", autoChooser);
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
      System.out.println("Starting auto");
      System.out.println(autoChooser.getSelected().toString());

      /*if(){
        drivetrain.setOdometryPosition(new Pose2d(1.3604378700256348, 5.526217937469482, new Rotation2d(3.1415)));
      }else if(autoChooser.getSelected().toString().equals("2NoteLeftAuto")){
        drivetrain.setOdometryPosition(new Pose2d(0.7523629665374756, 4.439711570739746, new Rotation2d(2.0943951023931)));
        
      }else if(autoChooser.getSelected().toString().equals("2NoteRightAuto")){
        drivetrain.setOdometryPosition(new Pose2d(0.7533434629440308, 6.666699409484863, new Rotation2d(-2.094395307179586)));
        
      }*/

      //return AutoBuilder.followPath(choreoTraj)
      //drivetrain.setOdometryPosition(new Pose2d(1.3604378700256348, 5.526217937469482, new Rotation2d(3.1415)));
      drivetrain.setOdometryPosition(new Pose2d(0.7533434629440308, 6.666699409484863, new Rotation2d(-2.094395307179586)));
      return AutoBuilder.buildAuto("2NoteRightAuto");
      //return AutoBuilder.buildAuto();
    } else{ return null;}

  }
}
