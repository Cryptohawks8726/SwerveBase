package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ArmSubsystem extends SubsystemBase implements BooleanSupplier {

    int deviceId = 50;
    int deviceId2 = 51;
    int shooterOffset = 135; // TODO: get real offset angle (around 135)
    double intakeAngle = toRads(6.98); // TODO: get real angles to intake/shoot
    double shootAngle = toRads(49); // at subwoofer
    double initVel = toRads(14); // TODO: get initial velocity of shooter
    double maxAmpAngle = 121.5;
    double speakerHeight = 5.9812; // in meters
    double absEncoderOffset = 124.7233920;
    // TODO: get constants
    double kp = 0.2;
    double ki = 0;
    double kd = 0;

    double ks = 0.25;
    double kg = 0.33; //0.33
    double kv = 3.62; //3.62
    double ka = 0; //0.02

    CANSparkMax motorController = new CANSparkMax(deviceId, MotorType.kBrushless); // motor controller
    CANSparkMax motorController2 = new CANSparkMax(deviceId2, MotorType.kBrushless); // secondary motor
    SparkAbsoluteEncoder absoluteEncoder = motorController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    
    Constraints constraints = new Constraints(0.3, 0.085); // TODO: find good constraints (in degrees/s)
    ProfiledPIDController pidController = new ProfiledPIDController(kp, ki, kd, constraints,0.2); // degrees
    ArmFeedforward armFF = new ArmFeedforward(ks, kg, kv, ka);
    State intakeState = new State(intakeAngle, 0);
    State shootState = new State(shootAngle, 0);
    State calculateAngleState = new State(shootAngle, 0); // will be changed based on calculated angle

    public ArmSubsystem() {
        absoluteEncoder.setPositionConversionFactor(360); // "192 rotations per 1 rotation"
        absoluteEncoder.setVelocityConversionFactor(360);
        absoluteEncoder.setZeroOffset(absEncoderOffset);
        absoluteEncoder.setInverted(true);

        motorController.setIdleMode(IdleMode.kCoast);
        motorController2.setIdleMode(IdleMode.kCoast);

        motorController.setInverted(true);
        motorController2.setInverted(false);

        pidController.setGoal(toRads(getArmAngle()));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Rad",getArmAngle());
        double pidOutput = pidController.calculate(toRads(getArmAngle()));
        State setpoint = pidController.getSetpoint();

        //double ff = armFF.calculate(setpoint.position-toRads(14), setpoint.velocity);
        double ff =  ks + kg*Math.cos(setpoint.position-toRads(14)) + kv*setpoint.velocity;
        motorController.setVoltage(pidOutput+ff);
        motorController2.setVoltage(pidOutput+ff);

        SmartDashboard.putNumber("armDeg", getArmAngle());
        SmartDashboard.putNumber("armVel", absoluteEncoder.getVelocity());
        SmartDashboard.putNumber("Applied Voltage", pidOutput+ff);
        SmartDashboard.putNumber("Set Pos",setpoint.position-toRads(14));
        SmartDashboard.putNumber("Goal", pidController.getGoal().position);
        SmartDashboard.putNumber("Set Velocity",setpoint.velocity);
        SmartDashboard.putNumber("FF", ff);
        SmartDashboard.putNumber("PID",pidOutput);
        SmartDashboard.putNumber("error",pidController.getPositionError()*180/Math.PI);
        SmartDashboard.putNumber("Setpoint diff",Math.abs(getArmAngle()-pidController.getGoal().position) );
        //SmartDashboard.putNumber("calculatedAngle", calculateAngle(pidOutput, ff))
    }

    @Override
    public boolean getAsBoolean() { // will check if the arm angle is at the setpoint periodically
        if (Math.abs(getArmAngleRad()-pidController.getGoal().position) < toRads(5)) {
            return true;
        } else {
            return false;
        }
    }

    public double toRads(double degrees) {
        return degrees * Math.PI / 180;
    }

    /**
     * in degrees
     */
    public double getArmAngle() {
        return absoluteEncoder.getPosition();
    }

    public double getArmAngleRad() {
        return toRads(absoluteEncoder.getPosition());
    }

    /**
     * in degrees
     */
    public double getShooterAngle() {
        return 180-(getArmAngle()+shooterOffset);
    }

    /**
     * in degrees
     */
    public Command rotateToIntake() { // sets the set point, angle to intake @ ground
        return new 
        InstantCommand(() -> pidController.setGoal(intakeAngle));
        /*
        .andThen(new WaitUntilCommand(this))
        .andThen(new InstantCommand(()->{
            motorController.setVoltage(0.0);
            motorController2.setVoltage(0.0);
            SmartDashboard.putString("At zero", "true");
        })); // in degrees */
    }

    /**
     * in degrees
     */
    public InstantCommand rotateToShoot() { // angle to shoot @ subwoofer
        return new InstantCommand(() -> pidController.setGoal(shootAngle)); // in degrees
    }

    /**
     * calculates the angle to shoot to the speaker at any field pos
     * @param posX the field position of the robot relative to the speaker
     * @return angle (in degrees)
     */
    double calculateAngle(double posX, double posY) { // position relative to subwoofer
        double horDist = Math.sqrt(posX*posX+posY*posY);

        // the angle that the shooter should be
        double shooterAngle = Math.atan(initVel + Math.sqrt(Math.pow(initVel, 4) - 9.81*(9.81*horDist*horDist + 2*speakerHeight*initVel*initVel)) / (9.81*horDist));
        
        double armAngle = 180 - shooterAngle - shooterOffset;
        
        // TODO: consider shooting at an angle a little more because the hood is pretty big
        return armAngle;
    }

    /**
     * calculates the angle to shoot to the speaker at any field pos and rotates the arm
     */
    public InstantCommand rotateToCalculatedAngle(double posX, double posY) {
        calculateAngleState.position = calculateAngle(posX, posY);

        return new InstantCommand(() -> pidController.setGoal(calculateAngleState)); // in degrees
    }
    
}
