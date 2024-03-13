// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
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
    private final SendableChooser<String> autoChooser;
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

      //autoChooser = AutoBuilder.buildAutoChooser();
      autoChooser = new SendableChooser<String>();
      autoChooser.addOption("2NoteCenterAuto","2NoteCenterAuto");
      autoChooser.addOption("2NoteRightAuto","2NoteRightAuto");
      autoChooser.addOption("2NoteLeftAuto","2NoteLeftAuto");
      autoChooser.addOption("3NoteRightAuto", "3NoteRightAuto");
      autoChooser.addOption("3NoteLeftAuto", "3NoteLeftAuto");
      configureBindings();

      SmartDashboard.putData("Auto Chooser", autoChooser);

      NamedCommands.registerCommand("ShootFirstNote", arm.rotateToState(new State(Math.toRadians(7.5), 0)).andThen(shooter.fireNote(false))); //shooter.fireNote(false) without remy
      NamedCommands.registerCommand("IntakeNoteCmd0", arm.rotateToState(new State(Math.toRadians(4.7), 0))
        .andThen(shooter.startIntake()));
      NamedCommands.registerCommand("IntakeNoteCmd3", arm.rotateToState(new State(Math.toRadians(4.7), 0))
        .andThen(shooter.startIntake()));
      NamedCommands.registerCommand("ShootSecondNote", arm.rotateToState(new State(Math.toRadians(7.5), 0)).andThen(shooter.fireNote(false)));
      NamedCommands.registerCommand("ShootThirdNote", arm.rotateToState(new State(Math.toRadians(7.5), 0)).andThen(shooter.fireNote(false)));
      
    }

    private void configureBindings() {

      drivetrain.setDefaultCommand(new ActualXboxTeleopDrive(drivetrain,driverController).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
      driverController.start().onTrue(drivetrain.resetGyroAngle());
      
      operatorController.leftTrigger()
          .onTrue(
              arm.rotateToState(Arm.tempIntakeState)
          .andThen(shooter.startIntake()));
      // .onFalse(shooter.stopShooter()); //stopShooter isn't functional atm
      operatorController.rightTrigger()
          .onTrue(
              new ConditionalCommand(
                  new PrintCommand("Already at angle"),
                  arm.rotateToState(Arm.tempShootState),
                  () -> arm.getArmDeg() > 40) // This checks if the Arm is likely going for the amp
          .andThen(shooter.fireNote(false)));
          //.onFalse(shooter.stopShooter());
      operatorController.leftBumper().onTrue(shooter.nudgeIntake());
      operatorController.povUp().onTrue(shooter.pullBackNote());
      //operatorController.rightBumper().onTrue(shooter.stopShooter());
      operatorController.a().onTrue(arm.rotateToState(Arm.intakeState));
      operatorController.b().onTrue(arm.rotateToState(Arm.ampState));
      operatorController.y().onTrue(arm.rotateToState(Arm.sourceState));

      operatorController.back().onTrue(climber.smartReleaseClimber());
      operatorController.start().onTrue(climber.smartClimb());
      
    }

    public Command getAutonomousCommand() {
      if(!Constants.demoMode){
      System.out.println("Starting auto");


      if(autoChooser.getSelected().equals("2NoteCenterAuto")){
        return AutoBuilder.buildAuto("2NoteCenterAuto");
      }else if(autoChooser.getSelected().equals("2NoteLeftAuto")){
        return AutoBuilder.buildAuto("2NoteLeftAuto");
      }else if(autoChooser.getSelected().equals("2NoteRightAuto")){
        return AutoBuilder.buildAuto("2NoteRightAuto");
      }else{
        return AutoBuilder.buildAuto("2NoteCenterAuto"); //default path to do if nothing is selected
      }
      
      

    } else{ return null;}

  }
}
