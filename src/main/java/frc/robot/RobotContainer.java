// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.SwerveCommandManager;
import frc.robot.State.StateVariables;
import frc.robot.util.stateStuff.StatefulSubsystem;

public class RobotContainer extends StatefulSubsystem {
  // Sent to NetworkTables to allow the drive team to select multiple auto
  // options.
  private final SendableChooser<Command> autoChooser;
  

  // Change directory to change the swerve config
  public final SwerveSubsystem swerve = new SwerveSubsystem(
          new File(Filesystem.getDeployDirectory(), "2025bunnybotSwerve"));
  // Reference the command manager when actually commanding the swerve
  public final SwerveCommandManager swerveCommander = new SwerveCommandManager(swerve);

  public final OdometrySubsystem apriltagVision = new OdometrySubsystem(swerve);

  public RobotContainer() {        
    super("Robot");

    autoChooser = new SendableChooser<>();

    autoChooser.addOption("ShootDaStuffLow", new InstantCommand(() -> {}));

    SmartDashboard.putData("Auto Chooser", autoChooser);
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

  public static void updateNetworkTables(RobotContainer robot) {
    SmartDashboard.putString("Current State", robot.getCurrentCommandName());

    SmartDashboard.putNumber("luniteCount", StateVariables.getLuniteCount());
    SmartDashboard.putNumber("gameTime", Timer.getMatchTime());

    Pose2d pose = robot.swerve.getPose();
    SmartDashboard.putNumberArray("robot2DPosition",
            new double[] { pose.getMeasureX().baseUnitMagnitude(), pose.getMeasureY().baseUnitMagnitude(),
                    pose.getRotation().getRadians() });
  }
}
