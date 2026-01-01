package frc.robot.states;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotContainer;
import frc.robot.util.StateBase;

//Currently matches Bunnybot State Diagram V2
public class ExampleState extends StateBase {
    public RobotContainer robot;

    public ExampleState(RobotContainer container) {
        robot = container;

        this.setName("Example State Name");
    }

    @Override  
    public void execute() {
        if (controller.getAButtonPressed()) robot.johnSubsystem.exampleCommand();
        if (controller.getBButtonPressed()) {
            System.out.println("hello?");
            robot.runNextCommand(new ExampleState2(robot), false);
        }
        if (controller.getXButtonPressed()) robot.runNextCommand(new PrintCommand("Test1"), false);
    }
}