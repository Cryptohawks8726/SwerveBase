package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.stateStuff.StatefulSubsystem;

public class ExampleSubsystem extends StatefulSubsystem {
    public final PIDController pidController = new PIDController(0, 0, 0);

    public ExampleSubsystem() {
        super("ExampleSubsystem");

        // Call this to publish the subsystem to NetworkTables
        // Make sure to call this for any subsystems that need to display info.
        putOnDashboard();
    }

    @Override
    public void periodic() {
    }

    public void exampleCommand() {
        runNextCommand(runOnce(() -> {
            SmartDashboard.putString("Example Field", "Command Executed At " + Timer.getFPGATimestamp());
        }).withName("Example Command"), false);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("kP", pidController::getP, null);
    }
}
