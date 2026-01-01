package frc.robot.states;

import frc.robot.RobotContainer;
import frc.robot.util.StateBase;

//Currently matches Bunnybot State Diagram V2
public class ExampleState extends StateBase {
    public RobotContainer internalContainer;

    public ExampleState(RobotContainer container) {
        internalContainer = container;

        this.setName("Example State Name");
    }

    @Override  
    public void execute() {
        if (controller.getAButtonPressed()) internalContainer.johnSubsystem.exampleCommand();
    }
}