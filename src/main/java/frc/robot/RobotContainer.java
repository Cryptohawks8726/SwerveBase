// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
  private final SwerveDrive drivetrain;

  // private final CommandXboxController driverController;
  
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;
  private final SendableChooser<String> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Unmanaged.setPhoenixDiagnosticsStartTime(-1);
    drivetrain = new SwerveDrive();
    
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    autoChooser = new SendableChooser<String>();
    autoChooser.setDefaultOption("EXAMPLE","EXAMPLE_AUTO_FILENAME");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    NamedCommands.registerCommand("EXAMPLE_COMMAND", new PrintCommand(""));

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
    driverController.rightBumper().whileTrue(drivetrain.passiveBrake());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if(!Constants.demoMode){
      if (autoChooser.getSelected() != null) {
        return AutoBuilder.buildAuto(autoChooser.getSelected());
      }else{
        return null;
      }
  } else{ return null;}
 }
}
