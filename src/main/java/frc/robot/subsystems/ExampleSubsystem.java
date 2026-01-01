package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.stateStuff.StatefulSubsystem;

public class ExampleSubsystem extends StatefulSubsystem {
    public ExampleSubsystem() { 
        super("ExampleSubsystem");

        SmartDashboard.putString("Example Field", "Command Not Executed");
    }

    @Override
    public void periodic() {}

    public void exampleCommand() {
        runNextCommand(runOnce(() -> {
            SmartDashboard.putString("Example Field", "Command Executed At " + Timer.getFPGATimestamp());
        }).withName("Example Command"), false);
    }
}
