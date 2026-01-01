package frc.robot.states;

import frc.robot.RobotContainer;
import frc.robot.util.StateBase;

//Currently matches Bunnybot State Diagram V2
public class ExampleState2 extends StateBase {
    public RobotContainer robot;

    public ExampleState2(RobotContainer container) {
        robot = container;

        this.setName("Example State Name 2");
    }

    @Override  
    public void execute() {
        if (controller.getAButtonPressed()) {}
        if (controller.getBButtonPressed()) robot.runNextCommand(new ExampleState(robot), false);

        super.execute();
    }
}