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
     * Identical to Command.execute(), with some extra code for handling
     * XboxController functionality
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

    }

    @Override
    public void end(boolean wasInterrupted) {
        // states don't really have interrupt behavior in this setup, they're always
        // just cancelled if another comes in to override them
        onStateExit();
    }
}
