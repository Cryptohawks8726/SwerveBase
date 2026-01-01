package frc.robot.util;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.stateStuff.XboxControllerWithTriggers;

/**
 * Base class used to define robot-wide state. Command modified to have easy 
 */
public class StateBase extends Command {
    private HashMap<BooleanSupplier, Runnable> bindings = new HashMap<>();
    public XboxControllerWithTriggers controller = new XboxControllerWithTriggers(0);

    public void addBinding(BooleanSupplier binding, Runnable runnable) {
        bindings.put(binding, runnable);
    }
}
