package frc.robot.subsystems;

import java.time.Instant;
import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase implements BooleanSupplier {

    int deviceId = 0;
    int shooterOffset = 135; // TODO: get real offset angle (around 135)
    double intakeAngle = 10; // TODO: get real angles to intake/shoot
    double shootAngle = 45; // at subwoofer

    double kp = 0;
    double ki = 0;
    double kd = 0;

    CANSparkMax motorController = new CANSparkMax(deviceId, MotorType.kBrushless); // motor controller
    // TODO: is this the right type???
    SparkAbsoluteEncoder absoluteEncoder = motorController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    Constraints constraints = new Constraints(10, 5); // TODO: find good constraints (in degrees/s)
    ProfiledPIDController pidController = new ProfiledPIDController(kp, ki, kd, constraints); // degrees

    State intakeState = new State(intakeAngle, 0);
    State shootState = new State(shootAngle, 0);
    State calculateAngleState = new State(shootAngle, 0); // will be changed based on calculated angle

    public ArmSubsystem() {

    }

    @Override
    public void periodic() {
        //motorController.set(0);
    }


    @Override
    public boolean getAsBoolean() { // will check if the arm angle is at the setpoint periodically
        if (Math.abs(getArmAngle()-pidController.getSetpoint().position) < 0.1) {
            return true;
        } else {
            return false;
        }
    }

    public double getArmAngle() {
        return absoluteEncoder.getPosition();
    }

    public double getShooterAngle() {
        return 180-(getArmAngle()+shooterOffset);
    }

    public InstantCommand rotateToIntake() { // sets the set point, angle to intake @ ground
        return new InstantCommand(() -> pidController.setGoal(intakeState)); // in degrees
    }

    public InstantCommand rotateToShoot() { // angle to shoot @ subwoofer
        return new InstantCommand(() -> pidController.setGoal(shootState)); // in degrees
    }

    // calculates the angle to shoot to the speaker at any field pos and rotates it
    double calculateAngle(double posX, double posY) { // position relative to subwoofer
        double dist2D = Math.sqrt(posX*posX+posY*posY);

        double shooterAngle = Math.atan(1.9812/dist2D) * (180/Math.PI); // the angle that the shooter should be
        
        double armAngle = 180 - shooterAngle - shooterOffset;
        
        // TODO: consider shooting at an angle a little more because the hood is pretty big
        return armAngle;
    }

    public InstantCommand rotateToCalculatedAngle() {
        calculateAngleState.position = calculateAngle(1, 1); // TODO: get real field pos values

        return new InstantCommand(() -> pidController.setGoal(calculateAngleState)); // in degrees
    }
    
}
