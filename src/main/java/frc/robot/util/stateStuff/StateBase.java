package frc.robot.util.stateStuff;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Base class used to define robot-wide state. Command modified to have easy 
 */
public abstract class StateBase extends Command {
    public static XboxControllerWithTriggers controller = new XboxControllerWithTriggers(0);
    
    /**
     * Identical to Command.initialize() 
     */
    public abstract void onStateEnter(); 

    /**
     * Identical to Command.execute(), with some extra code for handling XboxController functionality 
     */
    public abstract void periodic();

    /**
     * Identical to Command.end() 
     */
    public abstract void onStateExit();

    @Override
    public void initialize() {
        onStateEnter();
    }

    @Override
    public void execute() {
        periodic();
        
        // //For the "pressed" methods to work you need to have them be polled every frame, this is stupid but we can't really do anything about it
        // //If you don't do this then subsystems that don't check i.e. the A button will carry over their inputs to a state that does
        // //I.e. if you push a in state1 which has no a binding then switch to state2 which has an a binding then that a binding will trigger the moment you switch unless you do this
        controller.stateManagerPeriodic();
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

    @Override
    public void end(boolean wasInterrupted) {
        //states don't really have interrupt behavior in this setup, they're always just cancelled if another comes in to override them
        onStateExit();
    }
}
