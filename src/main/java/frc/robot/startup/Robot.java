// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.startup;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.robotState.RobotState;
import frc.robot.robotState.StateManager;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.VisionConstants;;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private RobotContainer robotContainer;

  // Auto selection interface for choreo
  private final SendableChooser<String> autoChooser;

  // Auto selection interface for command-based autos
  private final SendableChooser<Command> cmdAutoChooser;

  // Allows you to choose which type of auto selection interface you should use
  private final SendableChooser<Boolean> booleanAutoTypeChooser;

  // The autonomous routine selected for the match
  private Command m_autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Add any looping operations you wanna make outside of subsystem periodics()
    // addPeriodic(() -> {
    // /* john johnson */; //Really trystan??
    // }, 0.02);

    // Updates NetworkTables constantly
    addPeriodic(RobotContainer::updateNetworkTables, 0.02);

    booleanAutoTypeChooser = new SendableChooser<Boolean>();
    autoChooser = new SendableChooser<String>();
    cmdAutoChooser = new SendableChooser<Command>();

    //auto1
    cmdAutoChooser.addOption("Auto1", new InstantCommand(() -> {
      
    }));
    //auto2
    cmdAutoChooser.addOption("Auto2", new InstantCommand(() -> {
      
    }));

    //auto3
    cmdAutoChooser.addOption("Auto3", new InstantCommand(() -> {
      
    }));
    //Default auto
    cmdAutoChooser.setDefaultOption("Default Auto", new InstantCommand(() -> {
    }));

    // Choreo auto chooser not in use, code has been left but commented and unused.
    SmartDashboard.putData("autoChooser", cmdAutoChooser);

    // booleanAutoTypeChooser.addOption("Use Choreo", true);
    // booleanAutoTypeChooser.setDefaultOption("Use Commands", false);

    // SmartDashboard.putData("Select Desired Auto Chooser Type",
    // booleanAutoTypeChooser);
  }

  // Passes chosen autonomous routine to Robot.java
  public Command getAutonomousCommand() {
    if (booleanAutoTypeChooser.getSelected() == true) {
      if (autoChooser.getSelected() != null) {
        return AutoBuilder.buildAuto(autoChooser.getSelected());
      } else {
        return null;
      }
    } else {
      return cmdAutoChooser.getSelected();
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    StateManager.setState(RobotState.STATE1);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    m_autonomousCommand = getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
}
