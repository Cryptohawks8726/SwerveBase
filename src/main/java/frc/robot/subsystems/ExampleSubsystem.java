package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.FieldPointDisplay;
import frc.robot.util.stateStuff.StatefulSubsystem;

public class ExampleSubsystem extends StatefulSubsystem {
    public final PIDController pidController = new PIDController(0, 0, 0);
    public double controlEffort = 0.0;
    private Pose2d exampleSetpoint = new Pose2d();

    public ExampleSubsystem() {
        super("ExampleSubsystem");

        // Call this to publish the subsystem to NetworkTables
        // Make sure to call this for any subsystems that need to display info.
        putOnDashboard();
    }

    @Override
    public void periodic() {
        // Can run any typical periodic logic here...
        // This method will be called every 20ms
    }

    public void startExampleCommand() {
        // The state will run this command periodically.
        runNextCommand(run(() -> {
            controlEffort = pidController.calculate(0.0);
        }).withName("Example Command"), false);
    }

    public void exampleSetMethod(double val) {
        pidController.setSetpoint(val);
    }

    public double getOutputControlEffort() {
        return controlEffort;
    }

    public void setExampleSetpoint(Pose2d newPose) {
        exampleSetpoint = newPose;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("kP", pidController::getP, null);
        builder.addDoubleProperty("Ouput Control Effort", this::getOutputControlEffort, null);

        // This call registers a setpoint that can be easily modified from the dashboard
        // or glass.
        // You can use either lambdas or defined methods, as seen from the different
        // getter and setter.
        FieldPointDisplay.registerSetpoint("Example Setpoint", () -> exampleSetpoint, this::setExampleSetpoint);
    }
}
