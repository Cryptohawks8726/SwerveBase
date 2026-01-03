package frc.robot;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.util.stateStuff.StateBase;

//Provide static classes representing robot-wide states here
public class States {
    public static class ExampleState extends StateBase {
        public RobotContainer robot;

        public ExampleState(RobotContainer newRobot) {
            robot = newRobot;

            //The name of this state will be treated as the robot's state's name in dashboard outputs
            this.setName("Example State Name");
        }

        @Override
        public void onStateEnter() {
            //john johnson
        }

        @Override
        public void periodic() {
            if (controller.getAButtonPressed()) robot.johnSubsystem.exampleCommand();
            if (controller.getBButtonPressed()) {
                robot.runNextCommand(new ExampleState2(robot), false);
            }
            if (controller.getXButtonPressed()) robot.johnSubsystem.runNextCommand(new PrintCommand("Test1"), false);
        }

        @Override
        public void onStateExit() {
            //my name is yoshikage kira
        }
    }

    //Currently matches Bunnybot State Diagram V2
    public static class ExampleState2 extends StateBase {
        public RobotContainer robot;

        public ExampleState2(RobotContainer container) {
            robot = container;

            this.setName("Example State Name 2");
        }

        @Override
        public void onStateEnter() {
            //the fitnessgram pacer test
        }

        @Override
        public void periodic() {
            if (controller.getBButtonPressed()) robot.runNextCommand(new ExampleState(robot), false);
        }

        @Override
        public void onStateExit() {
            //long ago before time had a name
        }
    }
}
