package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.stateStuff.StatefulSubsystem;

public class ExampleSubsystem extends StatefulSubsystem {
    public String johnVariable = "Command Not Executed";

    @Override
    public void periodic() {
        SmartDashboard.putString("Example Field", johnVariable);
    }

    public void exampleCommand() {
        System.out.println("attempting example command");

        runNextCommand(run(() -> {
            SmartDashboard.putString("Example Field", "Command Executed At " + Timer.getFPGATimestamp());
        }).withName("Example Command"), false);
    }
}
