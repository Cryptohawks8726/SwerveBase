package frc.robot.util.stateStuff;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StatefulSubsystem extends SubsystemBase {
    /// MAY BE NULL!
    protected Command activeCommand = null;
    protected Trigger executionTrigger;
    protected String lastScheduledCommand = "None";

    public StatefulSubsystem(String newSubsystemName) {
        setName(newSubsystemName);
    }

    /**
     * Schedules the next command (state) to run on this subsystem. Only one command
     * will run at a time.
     * 
     * @param toRun            The command to run. Passing null will cancel the
     *                         current command.
     * @param runsWhenDisabled Whether or not the command should run when disabled.
     */
    public void runNextCommand(Command toRun, boolean runsWhenDisabled) {
        if (DriverStation.isDisabled() && runsWhenDisabled == false)
            return;

        if (getCurrentCommand() != null)
            getCurrentCommand().cancel();

        // Kill the old active command
        cancelCurrentCommand();

        // Set the new active command
        activeCommand = toRun;

        // safety check in case a null command is passed
        if (toRun == null) {
            return;
        }

        activeCommand.initialize();

        lastScheduledCommand = toRun.getName();
    }

    public void cancelCurrentCommand() {
        if (activeCommand != null) {
            activeCommand.end(true);
            activeCommand = null;

        }
    }

    /**
     * Retrieves the current command, if one is currently running.
     * 
     * @return The name of the command, if a name was provided. "None" if no command
     *         is running. The name of the command's class if it is unnamed.
     */
    public String getCurrentCommandName() {
        return activeCommand != null ? activeCommand.getName() : "None";
    }

    public String getLastScheduledCommandName() {
        return lastScheduledCommand;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // The command that is currently running.
        builder.addStringProperty("currentCommandName", this::getCurrentCommandName, null);
        // The last command that was scheduled. This command may or may not have since
        // ended.
        builder.addStringProperty("lastScheduledCommand", this::getLastScheduledCommandName, null);
    }

    public void putOnDashboard() {
        SmartDashboard.putData("Subsystems/" + getName(), this);
    }

    @Override
    public void periodic() {
        // Run the active command
        activeCommand.execute();
        if (activeCommand.isFinished()) {
            activeCommand.end(false);
            activeCommand = null;
        }

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