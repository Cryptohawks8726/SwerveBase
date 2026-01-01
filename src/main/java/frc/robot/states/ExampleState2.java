package frc.robot.states;

import frc.robot.RobotContainer;
import frc.robot.util.StateBase;

//Currently matches Bunnybot State Diagram V2
public class ExampleState2 extends StateBase {
    public RobotContainer internalContainer;

    public ExampleState2(RobotContainer container) {
        internalContainer = container;

        this.setName("Example State Name 2");
    }

    @Override  
    public void execute() {
    }
}