package frc.robot.util.stateStuff;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
            .handleInterrupt(() -> {
                activeCommand = null;
                currentCommandName = "None";
            })
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
            .ignoringDisable(true));
    }

    public void runNextCommand(Command toRun, boolean runsWhenDisabled) {
        if (DriverStation.isDisabled() && runsWhenDisabled == false) return; 

        if (getCurrentCommand() != null) getCurrentCommand().cancel();

        System.out.println(subsystemName + ", " + getCurrentCommandName() + ", " + getSupplierStatus());

        supplier = toRun;

        currentCommandName = toRun.getName();

        SmartDashboard.putString(subsystemName + " State", currentCommandName);
    }


    public String getCurrentCommandName() {
        return activeCommand != null ? currentCommandName : "None";
    }

    public String getSupplierStatus() {
        return supplier != null ? supplier.getName() : "None";
    }
}