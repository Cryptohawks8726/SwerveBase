package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.stateStuff.StateBase;

//Provide static classes representing robot-wide states here
public class State {
    /**
     * Variables which multiple states or subsystems need to access go here
     */
    public class StateVariables {
        public static int luniteCount = 0;
        public static boolean currentlyDetectingLunite = false;
        public static Pose2d shooterPoseSixSeven = new Pose2d();
        public static int shooterGoalTagSixSeven;

        public static void incrementLuniteCount() {
            if (luniteCount == 3)
                return;

            luniteCount++;
        }

        public static void decrementLuniteCount() {
            if (luniteCount == 0)
                return;

            luniteCount--;
        }

        public static void setLuniteCount(int newLuniteCount) {
            luniteCount = newLuniteCount;
        }

        public static int getLuniteCount() {
            return luniteCount;
        }
    }

    public static class ManualIntake extends StateBase {
        public RobotContainer robot;

        public ManualIntake(RobotContainer newRobot) {
            robot = newRobot;

            // The name of this state will be treated as the robot's state's name in
            // dashboard outputs
            setName("Manual Intake");
        }

        @Override
        public void onStateEnter() {
            // john johnson
            robot.swerve.setDefaultCommand(robot.swerveCommander.driveFieldOriented);
        }

        @Override
        public void periodic() {
            if (controller.getYButtonPressed()) {
                robot.runNextCommand(new ManualShoot(robot), false);
            }

            if (controller.getXButtonPressed()) {
                System.out.println("it works");
            }
        }

        @Override
        public void onStateExit() {
            // do funny stuff
        }
    }

    // Currently matches Bunnybot State Diagram V2
    public static class ManualShoot extends StateBase {
        public RobotContainer robot;
        private Command currentAlignmentCommand = null;
        private Command currentShootLunitesIndexerCommand = null;

        public ManualShoot(RobotContainer container) {
            robot = container;

            setName("Manual Shoot");
        }

        @Override
        public void onStateEnter() {
            robot.swerve.setDefaultCommand(robot.swerveCommander.slowDriveFieldOriented);
        }

        @Override
        public void periodic() {
            if (controller.getYButtonPressed()) {
                robot.runNextCommand(new ManualIntake(robot), false);
            }
            // So i just realized that all of this was for bunnybots thingy -> i think it
            // should be fine though but the actual pos stuff probably chages idk
            // i remember something about these pos being center based or something like
            // that idk

            if (controller.getXButtonPressed()) {
                // ermm the pos is the outer closer one assuming we are on blue alliance
                robot.swerve.driveToPose8726(
                        robot.apriltagVision.getTagRelativePose(new Pose2d(1.5, 0, new Rotation2d(Math.PI)), 18), 0.3);
            }

            if (controller.getAButtonPressed()) {
                robot.apriltagVision.simulateTagRelativeSetpoint(new Pose2d(1.5, 0, new Rotation2d(Math.PI)), 18);
            }

            if (controller.getBButtonPressed() && robot.swerve.getCurrentCommand() != null) {
                robot.swerve.getCurrentCommand().cancel();
                robot.swerve.setDefaultCommand(robot.swerveCommander.slowDriveFieldOriented); // Ermm this should cancel (in
                                                                                          // theory)
            }
        }

        @Override
        public void onStateExit() {
            if (currentAlignmentCommand != null)
                currentAlignmentCommand.cancel();

            if (currentShootLunitesIndexerCommand != null)
                currentShootLunitesIndexerCommand.cancel();
        }
    }
}
