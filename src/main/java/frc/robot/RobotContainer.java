// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.XboxTeleopDrive;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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
  
  private final CommandJoystick driverController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    
    drivetrain = new SwerveDrive();
    // driverController = new CommandXboxController(0);
    
    driverController = new CommandJoystick(0);

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
    ///drivetrain.setDefaultCommand(new XboxTeleopDrive(drivetrain,driverController).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    
    SmartDashboard.putNumber("FRVoltage", 0);

    Trigger trigger = driverController.button(1); // the "trigger" on the joystick increases voltage
    trigger.onTrue(new InstantCommand(()->{
      while(drivetrain.modules.get(0).wantedVoltage <= 2.5){
        drivetrain.modules.get(0).wantedVoltage += .0001;
        drivetrain.modules.get(0).increaseVoltage();
        SmartDashboard.putNumber("FRVoltage", drivetrain.modules.get(0).wantedVoltage);
        SmartDashboard.putNumber("encoderVelocity", drivetrain.modules.get(0).getVelocity());
      }
      ;}));
    trigger.whileFalse(new InstantCommand(()->{
      drivetrain.modules.get(0).wantedVoltage = 0;
      drivetrain.modules.get(0).noVoltage();
      SmartDashboard.putNumber("FRVoltage", drivetrain.modules.get(0).wantedVoltage);
      SmartDashboard.putNumber("encoderVelocity", drivetrain.modules.get(0).getVelocity());
      ;}));
    
    
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
  /*public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  } */
}
