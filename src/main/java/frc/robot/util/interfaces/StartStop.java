package frc.robot.util.interfaces;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Interface for subsystems with a component that can be started and stopped.
 */
public interface StartStop {
    /**
     * Starts the subsystem.
     */
    public void start();

    /**
     * Stops the subsystem.
     */
    public void stop();

    /**
     * Starts the subsystem for a set period of time, then stops it.
     * 
     * @param seconds Seconds to run the subsytem for
     */
    default public Command runFor(double seconds) {
        return new WaitCommand(seconds).beforeStarting(this::start).finallyDo(this::stop);
    }
}