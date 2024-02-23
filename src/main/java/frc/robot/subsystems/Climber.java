package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private CANSparkMax winchMotor;
    private RelativeEncoder winchEncoder;
    private double startRev;

    /*
     * Creates a climber subsystem
     * 
     * @return The climber subsystem
     */
    public Climber() {
        winchMotor = new CANSparkMax(Constants.Climber.winchMotorId, MotorType.kBrushless);
        winchMotor.enableVoltageCompensation(12.0);
        winchMotor.setSmartCurrentLimit(Constants.Climber.stallCurrentLimit, Constants.Climber.freeCurrentLimit);

        winchEncoder = winchMotor.getEncoder();
    }

    /*
     * Releases the climber to be fully extended by the CF spring by driving the
     * motor for a set time.
     * Warning: Untested.
     * 
     * @return A command to release the climber
     */
    public Command releaseClimber() {
        return new InstantCommand(() -> {
            winchMotor.setVoltage(3);
        })
                .andThen(new WaitCommand(1.0))
                .andThen(() -> winchMotor.setVoltage(0.0))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("Release Climber");
    }

    /**
     * Releases the climber via waiting for the motor to move a set number of
     * revolutions.
     * Warning: Untested.
     * 
     * @return A command to release the climber
     */
    public Command smartReleaseClimber() {
        return new InstantCommand(() -> {
            winchMotor.setVoltage(-1.5); // TODO verify voltage is high enough and direction is correct
            this.startRevCount();
        })
                .andThen(new WaitUntilCommand(() -> this.passedRevGoal(5))) // TODO verify goal works
                .finallyDo(() -> winchMotor.setVoltage(0.0))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("Smart Release Climber");
    }

    /**
     * Runs the climber at a set voltage until the encoder velocity drops below a
     * set limit.
     * 
     * @returns The climb command
     */
    public Command climb() {
        return new InstantCommand(() -> winchMotor.setVoltage(6.0))// TODO verify voltage is sufficient
                .andThen(new WaitCommand(0.8))
                .andThen(new WaitUntilCommand(() -> winchEncoder.getVelocity() < 0.25))// TODO verify velocity limit
                                                                                       // works
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                .finallyDo(() -> winchMotor.setVoltage(0.0))
                .withName("Climb");
    }

    /*
     * Records the encoder position at the time of calling
     */
    private void startRevCount() {
        startRev = winchEncoder.getPosition();
    }

    /*
     * Compares the number of revolutions since <code>startRevCount()</code> was
     * called to the goal.
     * 
     * @param goal the number of revolutions that should be passed
     * 
     * @return True if the number of revolutions passed is >= to the goal
     */
    private boolean passedRevGoal(double goal) {
        return (startRev - winchEncoder.getPosition()) >= goal;
    }

}
