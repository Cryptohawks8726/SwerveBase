package frc.robot.util.stateStuff;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StatefulSubsystem extends SubsystemBase {
    protected Command supplier = null;
    protected Command activeCommand = null;
    protected String subsystemName = "UnnamedSubsystem";
    protected Trigger executionTrigger;
    protected String currentCommandName = "None";

    public StatefulSubsystem(String newSubsystemName) {
        subsystemName = newSubsystemName;

        executionTrigger = new Trigger(() -> supplier != null);

        executionTrigger
                .onTrue(defer(() -> runOnce(() -> {
                    activeCommand = supplier;
                    supplier = null;
                })
                        .andThen(defer(() -> {
                            activeCommand.addRequirements(this);
                            return activeCommand;
                        })))
                        .ignoringDisable(true));
    }

    public void runNextCommand(Command toRun, boolean runsWhenDisabled) {
        if (DriverStation.isDisabled() && runsWhenDisabled == false)
            return;

        if (getCurrentCommand() != null)
            getCurrentCommand().cancel();

        supplier = toRun;

        currentCommandName = toRun.getName();

        SmartDashboard.putString(subsystemName + " State", currentCommandName);
    }

    /**
     * Retrieves the current command, if one is currently running.
     * 
     * @return The name of the command, if a name was provided. "None" if no command
     *         is running. The name of the command's class if it is unnamed.
     */
    public String getCurrentCommandName() {
        return activeCommand != null ? currentCommandName : "None";
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("currentCommandName", this::getCurrentCommandName, null);
    }
}