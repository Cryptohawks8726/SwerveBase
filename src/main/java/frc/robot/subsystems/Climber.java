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
    public Climber(){
        winchMotor = new CANSparkMax(Constants.Climber.winchMotorId, MotorType.kBrushless);
        winchMotor.enableVoltageCompensation(12.0);
        winchMotor.setSmartCurrentLimit(Constants.Climber.stallCurrentLimit, Constants.Climber.freeCurrentLimit);

        winchEncoder = winchMotor.getEncoder();
        //winchMotor.enableSoftLimit(null, false)
    }

    public Command releaseClimber(){
        return new InstantCommand(()->{winchMotor.setVoltage(3);})
        .andThen(new WaitCommand(1.0))
        .andThen(()->winchMotor.setVoltage(0.0))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Release Climber")
        ;
    }
    /**
     * 
     * @return
     */
    public Command smartReleaseClimber(){
        return new InstantCommand(()->{
            winchMotor.setVoltage(-1.5);
            this.startRevCount();
        })
        .andThen(new WaitUntilCommand(()-> this.passedRevGoal(5))) //This value needs to be verified and put into Constants.java
        .finallyDo(()->winchMotor.setVoltage(0.0))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Smart Release Climber")
        ;
    }

    /**
     * Runs the climber at a set voltage until the encoder velocity drops below a set limit.
     * @returns {Command} The climb command
     */
    public Command climb(){
        return new InstantCommand(()->winchMotor.setVoltage(6.0))
        .andThen(new WaitCommand(0.8))
        .andThen(new WaitUntilCommand(()-> winchEncoder.getVelocity() < 0.25))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .finallyDo(()-> winchMotor.setVoltage(0.0))
        .withName("Climb");
    }

    private void startRevCount(){
        startRev = winchEncoder.getPosition();
    }

    private boolean passedRevGoal(double goal){
        return (startRev-winchEncoder.getPosition()) > goal;
    }

}
