package frc.robot.robotState;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.XboxControllerWithTriggers;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
//Pretty sure this does state stuff :)
//state thingy magingy
public class StateManager {

    private static RobotState currentState;
    private static Command currentCommand;

    private static Map<RobotState, Command> stateCommands;

    public StateManager(Map<RobotState, Command> stateCommands) {
        //idk if this hates itself or not
        this.stateCommands = stateCommands;
    }

    public static void setState(RobotState newState) {

        System.out.println("STATE CHANGE FROM " + currentState.name() + " TO " + newState.name());
        //SHOULD in theory run the on exit command around here because it is a functional command and the state is being changed
        //I checked docs => calling cancel does run the end() part of the command
        if (currentCommand != null) {
            currentCommand.cancel();
        }
        //cool thingy that changes the state and also gets the new command from the map
        currentState = newState;
        currentCommand = stateCommands.get(newState);
        //If should be redundant check but whatever
        if (currentCommand != null) {
            currentCommand.schedule();
        }
    }
    //cool thingy ig
    public static RobotState getRobotState() {
        return currentState;
    }
}