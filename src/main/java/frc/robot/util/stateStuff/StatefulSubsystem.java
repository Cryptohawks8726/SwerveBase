package frc.robot.util.stateStuff;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StatefulSubsystem extends SubsystemBase {
    public Command supplier = null;

    public Trigger executingTrigger = new Trigger(() -> supplier != null);

    public StatefulSubsystem() {
        executingTrigger.onTrue(defer(() -> supplier.andThen(runOnce(() -> supplier = null))).ignoringDisable(true));

        runNextCommand(new PrintCommand("FINAL HELP").withName("John Test"), true);
    }

    public void runNextCommand(Command toRun, boolean runsWhenDisabled) {
        supplier = toRun.andThen(runOnce(() -> supplier = null))
            .withName(toRun.getName())
            .ignoringDisable(runsWhenDisabled);
    }
}