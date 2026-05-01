// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.FieldPointDisplay;
import frc.robot.util.SwerveCommandManager;
import frc.robot.util.stateStuff.StateBase;
import frc.robot.util.stateStuff.StatefulSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

public class RobotContainer extends StatefulSubsystem {
  // Sent to NetworkTables to allow the drive team to select multiple auto
  // options.
  private final SendableChooser<Command> autoChooser;

  // Change directory to change the swerve config
  public final SwerveSubsystem swerve = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "2025bunnybotSwerve"));
  // Reference the command manager when actually commanding the swerve
  public final SwerveCommandManager swerveCommander = new SwerveCommandManager(swerve);
  public final ExampleSubsystem johnSubsystem = new ExampleSubsystem();

  public RobotContainer() {
    super("Robot");

    autoChooser = new SendableChooser<>();

    autoChooser.addOption("ShootDaStuffLow", new InstantCommand(() -> {
    }));

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("FieldPointDisplay", new FieldPointDisplay());
    putOnDashboard();

    runNextCommand(new State.ManualIntake(this), true);
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

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    // Game time (seconds)
    builder.addDoubleProperty("gameTime", () -> Timer.getMatchTime(), null);

    Pose2d pose = swerve.getPose();
    // X, Y, Rotation (meters, meters, radians)
    builder.addDoubleArrayProperty("robotPosition", () -> new double[] { pose.getMeasureX().baseUnitMagnitude(),
        pose.getMeasureY().baseUnitMagnitude(), pose.getRotation().getRadians() }, null);
    builder.addBooleanProperty("zeroGyroTrigger", () -> false, (v) -> {
      // Reset the gyro if this value is set to true.
      if (v) {
        swerve.zeroGyroWithAlliance();
      }
    });
  }

  @Override
  public void periodic() {
    super.periodic();

    // For the "pressed" methods to work you need to have them be polled every
    // frame, this is stupid but we can't really do anything about it
    // If you don't do this then subsystems that don't check i.e. the A button
    // will carry over their inputs to a state that does
    // I.e. if you push a in state1 which has no a binding then switch to state2
    // which has an a binding then that a binding will trigger the moment you switch
    // unless you do this
    StateBase.controller.stateManagerPeriodic();
    StateBase.controller.getAButtonPressed();
    StateBase.controller.getBButtonPressed();
    StateBase.controller.getXButtonPressed();
    StateBase.controller.getYButtonPressed();
    StateBase.controller.getLeftTriggerPressed();
    StateBase.controller.getRightTriggerPressed();
    StateBase.controller.getLeftBumperButtonPressed();
    StateBase.controller.getRightBumperButtonPressed();
    StateBase.controller.getStartButtonPressed();
    StateBase.controller.getBackButtonPressed();
  }
}
