// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {
  CommandXboxController operatorController;
  ArmSubsystem armSubsystem;

  public RobotContainer() {
    operatorController = new CommandXboxController(0);
    armSubsystem = new ArmSubsystem();

    configureBindings();
  }

  private void configureBindings() {
    operatorController.a().onTrue(armSubsystem.rotateToIntake());
    operatorController.b().onTrue(armSubsystem.rotateToShoot());

    operatorController.leftBumper().onTrue(armSubsystem.rotateToCalculatedAngle());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
