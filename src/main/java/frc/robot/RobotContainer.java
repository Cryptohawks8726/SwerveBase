// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.states.ExampleState;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.stateStuff.StatefulSubsystem;

public class RobotContainer extends StatefulSubsystem {
  // Sent to NetworkTables to allow the drive team to select multiple auto
  // options.
  private final SendableChooser<Command> autoChooser;

  public final ExampleSubsystem johnSubsystem = new ExampleSubsystem();

  public RobotContainer() {        
    autoChooser = new SendableChooser<>();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    runNextCommand(new ExampleState(this), true);

    SmartDashboard.putBoolean("Command Null?", true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Perceived Robot State", supplier == null ? "None" : supplier.getName());
    SmartDashboard.putString("Robot State", getCurrentCommand() == null ? "None" : getCurrentCommand().getName());
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
