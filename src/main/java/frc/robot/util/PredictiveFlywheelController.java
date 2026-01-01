package frc.robot.util;

public class PredictiveFlywheelController {
    private double kV = 0;
    private double kS = 0;
    private double velocitySetpoint = 0;
    private double errorWindow = 0;
    private double drivingVoltage = 0;

    /**
     * Constructs a new PFC.
     * <p>
     * 
     * @param kV Feedforward constant that accounts for back-EMF
     * @param kS Feedforward constant that accounts for friction
     */
    public PredictiveFlywheelController(double newkV, double newkS) {
        kV = newkV;
        kS = newkS;
    }

    /**
     * Sets the desired velocity of the flywheel and parameters for getting there
     * 
     * @param newSetpoint       The new velocity setpoint for the flywheel, using
     *                          the same units as your feedforward
     * @param newErrorWindow    The error from the setpoint at which the controller
     *                          can deem the system to be "at rest"
     * @param newDrivingVoltage The driving voltage the controller will apply to
     *                          reach the setpoint
     */
    public void setSetpointParameters(double newSetpoint, double newErrorWindow, double newDrivingVoltage) {
        velocitySetpoint = newSetpoint;
        errorWindow = newErrorWindow;
        drivingVoltage = newDrivingVoltage;
    }

    /**
     * Calculates the optimal voltage for the flywheel in accordance with the last
     * setSetpointParameters call
     * 
     * @param currentVelocity The real-time velocity of the flywheel
     * @return The optimal voltage
     */
    public double calculate(double currentVelocity) {
        double error = velocitySetpoint - currentVelocity;

        if (Math.abs(error) < errorWindow)
            return kV * velocitySetpoint + Math.copySign(kS, currentVelocity);
        else
            return Math.copySign(drivingVoltage, error);
    }
}