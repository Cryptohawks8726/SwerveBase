package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.stateStuff.XboxControllerWithTriggers;

/**
 * Base class used to define robot-wide state. Command modified to have easy 
 */
public class StateBase extends Command {
    public static XboxControllerWithTriggers controller = new XboxControllerWithTriggers(0);
    
    @Override
    public void execute() {
        controller.stateManagerPeriodic();

        // //For the "pressed" methods to work you need to have them be polled every frame, this is stupid but we can't really do anything about it
        // //If you don't do this then subsystems that don't check i.e. the A button will carry over their inputs to a state that does
        // //I.e. if you push a in state1 which has no a binding then switch to state2 which has an a binding then that a binding will trigger the moment you switch unless you do this
        controller.getAButtonPressed();
        controller.getBButtonPressed();
        controller.getXButtonPressed();
        controller.getYButtonPressed();
        controller.getLeftTriggerPressed();
        controller.getRightTriggerPressed();
        controller.getLeftBumperButtonPressed();
        controller.getRightBumperButtonPressed();
        controller.getStartButtonPressed();
        controller.getBackButtonPressed();
    }
}
