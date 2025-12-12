package frc.robot.util.interfaces;

import edu.wpi.first.units.Unit;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An interface for anything controlled by a setpoint. This includes PID
 * controllers, feed forward controllers,
 * and anything else that tries to bring a system to a desired setpoint.
 * 
 * @param U The type of unit used for the setpoint - AngleUnit, DistanceUnit,
 *          etc.
 */
public interface SetpointControlled<U extends Unit> {

    /**
     * Activates the controller. If it was previously paused, the controller will
     * now attempt to bring the system to the desired setpoint.
     */
    public void startSetpointController();

    /**
     * Stops the controller. The controller will no longer attempt to move the
     * system.
     */
    public void stopSetpointController();

    /**
     * Sets the setpoint of the system. Note that it may not start moving unless the
     * controller was previously started using startSetpointController.
     * 
     * @param newSetpoint The desired setpoint
     */
    public void setSetpoint(U newSetpoint);

    /**
     * True if the system is deemed close enough to be at the setpoint.
     */
    public boolean isAtSetpoint();

    /**
     * Gets the error from the setpoint.
     * 
     * @return The error value, as a WPILib unit.
     */
    public U getSetpointError();

    /**
     * Gets an object implementing Sendable which can be published to NetworkTables
     * to view and change constants for the controller at runtime.
     * Most of the controllers in WPILib extends sendable, such as PIDController.
     * However, you can write your own implementation if needed.
     * 
     * @return The Sendable object
     */
    public Sendable getSendableController();

    /**
     * Will publish the controller in SmartDashboard under a path determined by the
     * names provided.
     * 
     * The specific path used is subsystemName/controllerName.
     * 
     * @param subsystemName  The name of the subsystem.
     * @param controllerName The name to give this controller.
     */
    default public void sendControllerToSmartDashboard(String subsystemName, String controllerName) {
        SmartDashboard.putData(subsystemName + '/' + controllerName, getSendableController());
    }

    // TODO: Add convient methods/commands

}
