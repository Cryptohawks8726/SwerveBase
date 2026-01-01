package frc.robot.startup;

import java.io.File;
import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.robotState.RobotState;
import frc.robot.robotState.StateManager;
import frc.robot.states.States;
import frc.robot.subsystems.*;
import frc.robot.util.SwerveCommandManager;

public class RobotContainer {

    private final XboxController controller = new XboxController(0);

    private final Subsystem1 Subsystem1 = new Subsystem1();
    private final Subsystem2 Subsystem2 = new Subsystem2();
    private final Subsystem3 Subsystem3 = new Subsystem3();

    //Cool trystan stuff :)
    public static final SwerveSubsystem swerve = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "2025bunnybotSwerve"));
    // Reference the command manager when actually commanding the swerve
    public static final SwerveCommandManager swerveCommander = new SwerveCommandManager(swerve);
    
    // Computer Vision Subsystems
    public static final VisionSubsystem apriltagVision = new VisionSubsystem(swerve);
    

    private StateManager stateManager;

    public RobotContainer() {
        stateManager = new StateManager(
            Map.of(
                RobotState.STATE1, States.state1(controller, Subsystem1, Subsystem2),
                RobotState.STATE2, States.state2(controller, Subsystem2, Subsystem3),
                RobotState.STATE3, States.state3(controller, Subsystem1, Subsystem3)
            )
        );

        // starting state
        StateManager.setState(RobotState.STATE1);
    }
    //ALL NETWORK TABLE UPDATES GO IN HERE
    //PLEASE DON'T SCATTER THEM LIKE WE DID AT BBOTS IM PRETTY SURE THE WEB UI PEOPLE DIDNT LIKE THAT
    public static void updateNetworkTables() {
        SmartDashboard.putString("currentState", StateManager.getRobotState().name());
        SmartDashboard.putNumber("gameTime", Timer.getMatchTime());
        
        Pose2d pose = RobotContainer.swerve.getPose();
        SmartDashboard.putNumberArray("robot2DPosition",
                new double[] { pose.getMeasureX().baseUnitMagnitude(), pose.getMeasureY().baseUnitMagnitude(),
                        pose.getRotation().getRadians() });


    }
}
