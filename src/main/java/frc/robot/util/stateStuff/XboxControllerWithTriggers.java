package frc.robot.util.stateStuff;
import edu.wpi.first.wpilibj.XboxController;

public class XboxControllerWithTriggers extends XboxController {
    private XboxController internalController;
    private boolean leftTriggerHeld = false;
    private boolean leftTriggerPressed = false;
    private boolean rightTriggerHeld = false;
    private boolean rightTriggerPressed = false;

    public XboxControllerWithTriggers(int port) {
        super(port);

        internalController = new XboxController(port);
    }

    public void stateManagerPeriodic() {
        boolean previousLeftTriggerHeld = leftTriggerHeld;
        boolean previousRightTriggerHeld = rightTriggerHeld;

        leftTriggerHeld = internalController.getLeftTriggerAxis() > 0.4;

        leftTriggerPressed = leftTriggerHeld != previousLeftTriggerHeld && leftTriggerHeld;

        rightTriggerHeld = internalController.getRightTriggerAxis() > 0.4;

        rightTriggerPressed = rightTriggerHeld != previousRightTriggerHeld && rightTriggerHeld;
    }

    public boolean getLeftTriggerPressed() {
        return leftTriggerPressed;
    }

    public boolean getLeftTriggerHeld() {
        return leftTriggerHeld;
    }

    public boolean getRightTriggerPressed() {
        return rightTriggerPressed;
    }

    public boolean getRightTriggerHeld() {
        return rightTriggerHeld;
    }
}