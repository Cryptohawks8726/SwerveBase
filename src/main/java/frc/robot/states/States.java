package frc.robot.states;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import frc.robot.robotState.RobotState;
import frc.robot.robotState.StateManager;
import frc.robot.subsystems.Subsystem1;
import frc.robot.subsystems.Subsystem2;
import frc.robot.subsystems.Subsystem3;

public class States {
    public static Command state1(XboxController controller, Subsystem1 subsystem1, Subsystem2 subsystem2) {
        return new FunctionalCommand(
            // onEnter
            () -> {
                //do the onEnter stuff here
            },

            // periodic
            () -> {
                //do periodic stuff here
                subsystem1.coolFunction();

                // bindings
                if (controller.getAButtonPressed()) {
                    StateManager.setState(RobotState.STATE2);
                }
            },

            // onExit
            (state1_ending) -> {
                
            },

            // finish condition => always should be the current state not being this state
            () -> {
                return StateManager.getRobotState() != RobotState.STATE1;
            }
        );
    }

    public static Command state2(XboxController controller, Subsystem2 subsystem2, Subsystem3 subsystem3) {
        return new FunctionalCommand(
            // onEnter
            () -> {
                //do the onEnter stuff here
            },

            // periodic
            () -> {
                //do periodic stuff here
                subsystem2.coolFunction();

                // bindings
                if (controller.getAButtonPressed()) {
                    StateManager.setState(RobotState.STATE3);
                }
            },

            // onExit
            (state2_ending) -> {
                
            },

            // finish condition => always should be the current state not being this state
            () -> {
                return StateManager.getRobotState() != RobotState.STATE2;
            }
        );
    }

    public static Command state3(XboxController controller, Subsystem1 subsystem1, Subsystem3 subsystem3) {
        return new FunctionalCommand(
            // onEnter
            () -> {
                //do the onEnter stuff here
            },

            // periodic
            () -> {
                //do periodic stuff here
                subsystem3.coolFunction();

                // bindings
                if (controller.getAButtonPressed()) {
                    StateManager.setState(RobotState.STATE1);
                }
            },

            // onExit
            (state3_ending) -> {
                
            },

            // finish condition => always should be the current state not being this state
            () -> {
                return StateManager.getRobotState() != RobotState.STATE3;
            }
        );
    }
}
