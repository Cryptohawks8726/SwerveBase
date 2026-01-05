// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.stateStuff.StatefulSubsystem;

public class RobotContainer extends StatefulSubsystem {
  // Sent to NetworkTables to allow the drive team to select multiple auto
  // options.
  private final SendableChooser<Command> autoChooser;

  public final ExampleSubsystem johnSubsystem = new ExampleSubsystem();

  // public final SwerveSubsystem swerve = new SwerveSubsystem(
  //           new File(Filesystem.getDeployDirectory(), "johnSwerve")); //TODO: UPDATE SWERVE CONFIGS

  public RobotContainer() {        
    super("Robot");

    autoChooser = new SendableChooser<>();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    runNextCommand(new State.ExampleState(this), true);
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (!Constants.demoMode) {
      return autoChooser.getSelected();
    } else {
      return null;
    }
  }
}
