package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase implements BooleanSupplier {

    int deviceId = 50;
    int deviceId2 = 51; // secondary motor
    int shooterOffset = 135; // TODO: get real offset angle (around 135)
    double intakeRad = toRads(6.98); // TODO: get real angles to intake/shoot
    double shootRad = toRads(49); // at subwoofer
    double shooterVel = 14; // TODO: get initial velocity of shooter
    double maxAmpAngle = 121.5;
    double speakerHeight = 5.9812; // in meters
    double absEncoderOffset = 124.7233920;
    // TODO: get constants
    double kp = 0.5;
    double ki = 0;
    double kd = 0;

    double ks = 0.135;
    double kg = 0.33; //0.33
    double kv = 3.62; //3.62
    double ka = 0; //0.02

    CANSparkMax motorController = new CANSparkMax(deviceId, MotorType.kBrushless); // motor controller
    CANSparkMax motorController2 = new CANSparkMax(deviceId2, MotorType.kBrushless); // secondary motor
    SparkAbsoluteEncoder absoluteEncoder = motorController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    
    Constraints constraints = new Constraints(0.3, 0.085); // TODO: find good constraints (in degrees/s)
    PIDController pidController = new PIDController(kp, ki, kd); // degrees
    ArmFeedforward armFF = new ArmFeedforward(ks, kg, kv, ka);
    State intakeState = new State(intakeRad, 0);
    State shootState = new State(shootRad, 0);
    State calculateAngleState = new State(shootRad, 0); // will be changed based on calculated angle

    State goal = new State();
    TrapezoidProfile trapezoidProfile = new TrapezoidProfile(constraints);

    public ArmSubsystem() {
        absoluteEncoder.setPositionConversionFactor(360); // "192 rotations per 1 rotation"
        absoluteEncoder.setVelocityConversionFactor(360);
        absoluteEncoder.setZeroOffset(absEncoderOffset);
        absoluteEncoder.setInverted(true);

        motorController.setIdleMode(IdleMode.kCoast);
        motorController2.setIdleMode(IdleMode.kCoast);

        motorController.setInverted(true);
        motorController2.setInverted(false);
        
        goal = new State(getArmRad(),0);
        pidController.setSetpoint(getArmRad());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Rad",getArmRad());
        pidController.setSetpoint(goal.position);
        double pidOutput = pidController.calculate(getArmRad());
        State setpoint = trapezoidProfile.calculate(0.02, new State(getArmRad(),toRads(absoluteEncoder.getVelocity())), goal);

        double ff = armFF.calculate(setpoint.position-toRads(0), setpoint.velocity);
        //double ff =  ks + kg*Math.cos(setpoint.position-toRads(14)) + kv*setpoint.velocity;
        motorController.setVoltage(pidOutput+ff);
        motorController2.setVoltage(pidOutput+ff);

        SmartDashboard.putNumber("armDeg", getArmDeg());
        SmartDashboard.putNumber("armVelDeg", absoluteEncoder.getVelocity());
        SmartDashboard.putNumber("Applied Voltage", pidOutput+ff);
        SmartDashboard.putNumber("Set Pos",setpoint.position-toRads(14));
        SmartDashboard.putNumber("Goal", goal.position);
        SmartDashboard.putNumber("Set Velocity",setpoint.velocity);
        SmartDashboard.putNumber("FF", ff);
        SmartDashboard.putNumber("PID",pidOutput);
        SmartDashboard.putNumber("error",pidController.getPositionError());
        SmartDashboard.putNumber("Setpoint diff",Math.abs(getArmRad()-goal.position) );
        //SmartDashboard.putNumber("calculatedAngle", calculateAngle(pidOutput, ff))
    }

    @Override
    public boolean getAsBoolean() { // will check if the arm angle is at the setpoint periodically
        if (Math.abs(getArmRad()-goal.position) < toRads(5)) {
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
    public double getArmDeg() {
        return absoluteEncoder.getPosition();
    }

    public double getArmRad() {
        return toRads(absoluteEncoder.getPosition());
    }

    /**
     * in degrees
     */
    public double getShooterAngle() {
        return 180-(getArmDeg()+shooterOffset);
    }

    /**
     * in degrees
     */
    public Command rotateToIntake() { // sets the set point, angle to intake @ ground
        return new 
        InstantCommand(() -> goal = new State(intakeRad,0));
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
        return new InstantCommand(() -> goal = new State(shootRad,0)); // in degrees
    }

    /**
     * calculates the angle to shoot to the speaker at any field pos
     * @param posX the field position of the robot relative to the speaker
     * @return angle (in degrees)
     */
    double calculateAngle(double posX, double posY) { // position relative to subwoofer
        double horDist = Math.sqrt(posX*posX+posY*posY);

        // the angle that the shooter should be
        double shooterAngle = Math.atan(shooterVel + Math.sqrt(Math.pow(shooterVel, 4) - 9.81*(9.81*horDist*horDist + 2*speakerHeight*shooterVel*shooterVel)) / (9.81*horDist));
        
        double armAngle = 180 - Math.toDegrees(shooterAngle) - shooterOffset;
        
        // TODO: consider shooting at an angle a little more because the hood is pretty big
        return armAngle;
    }

    /**
     * calculates the angle to shoot to the speaker at any field pos and rotates the arm
     */
    public InstantCommand rotateToCalculatedAngle(double posX, double posY) {
        calculateAngleState.position = calculateAngle(posX, posY);

        return new InstantCommand(() -> goal = calculateAngleState); // in degrees
    }
    
}
