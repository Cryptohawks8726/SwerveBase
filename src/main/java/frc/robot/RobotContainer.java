// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Constants;

// TODO: Rename stuff? Fix things? Improvemments?

public class RobotContainer extends SubsystemBase {
  // Sent to NetworkTables to allow the drive team to select multiple auto
  // options.
  private final SendableChooser<Command> autoChooser;

  // The next queued command and the currently running command.
  Command nextCommand = null;
  Command activeCommand = null;

  final Trigger commandQueueTrigger = new Trigger(() -> nextCommand != null);
  // tracked as a variable so we can change its name.
  final Command deferredSwitchCommand = new SequentialCommandGroup(
      new InstantCommand(() -> {
        activeCommand = nextCommand;
        nextCommand = null;
      }),
      defer(() -> activeCommand),
      new InstantCommand(() -> activeCommand = null)).withName("Unnamed RobotContainer Deferred Wrapper");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    autoChooser = new SendableChooser<>();
    SmartDashboard.putData(autoChooser);

    commandQueueTrigger.onTrue(deferredSwitchCommand);

    // This should usually be something.
    setDefaultCommand(new InstantCommand().withName("Empty default command - replace me!"));
  }

  /**
   * Schedules another command to run across the whole robot. Will cancel the
   * current one. Please ensure these commands are named! It makes it easier to
   * track robot logic.
   * 
   * @param toRun The command to run.
   */
  public void runNextCommand(Command toRun) {
    toRun.addRequirements(this);
    stopCurrentCommand();
    deferredSwitchCommand.setName(toRun.getName());
    nextCommand = toRun; // will make the trigger go off.
  }

  /**
   * Stops the actively running command. (just calls getCurrentCommand().cancel())
   */
  public void stopCurrentCommand() {
    getCurrentCommand().cancel();
  }

  /**
   * Gets the name of the current command.
   * 
   * @return The name of the command currently running on RobotContainer.
   */
  public String getCurrentCommandName() {
    Command cmd = getCurrentCommand();
    if (cmd != null) {
      return cmd.getName();
    } else {
      return "None"; // is null preferred? probably not since java nulls SUCK
    }
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
