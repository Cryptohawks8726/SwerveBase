package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Arm;
import frc.robot.Constants.Field;

public class ArmSubsystem extends SubsystemBase{

    int shooterOffset = 135; // TODO: get real offset angle (around 135)

    double shooterVel = 14; // TODO: get initial velocity of shooter
    double maxAmpAngle = 121.5;
   
    double kp = 2.0;//
    double ki = 0;
    double kd = 0;

    double ks = 0.135;
    double kg = 0.22; //0.33
    double kv = 3.62; //3.62
    double ka = 0; //0.02
    private final double DEG_TO_RAD = 0.017453292519943295; 
    
    private final CANSparkMax motorController = new CANSparkMax(Arm.rightMotorId, MotorType.kBrushless);
    private final CANSparkMax motorController2 = new CANSparkMax(Arm.leftMotorId, MotorType.kBrushless);
    private final SparkAbsoluteEncoder absoluteEncoder = motorController.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    
    private final Constraints constraints = new Constraints(Arm.maxVelRads, Arm.maxAccelRads); // TODO: find good constraints (in rad/s)
    private final PIDController pidController = new PIDController(kp, ki, kd); // degrees
    private final ArmFeedforward armFF = new ArmFeedforward(ks, kg, kv, ka);
    private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(constraints);

    private State calculateAngleState = new State(0, 0); // will be changed based on calculated angle
    private State goal = new State(); // will be changed to reflect current goal
    private double t;

    public ArmSubsystem() {
        absoluteEncoder.setPositionConversionFactor(360);
        absoluteEncoder.setVelocityConversionFactor(360);
        absoluteEncoder.setZeroOffset(Arm.absEncoderOffset);
        absoluteEncoder.setInverted(true);

        motorController.setIdleMode(IdleMode.kCoast); 
        motorController2.setIdleMode(IdleMode.kCoast);

        motorController.setInverted(true);
        motorController2.setInverted(false);
        
        goal = new State(getArmRad(),0);
        pidController.setSetpoint(getArmRad());
        t = 0.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isAtAmp", this.atStatePos(Arm.ampState));
        t+=0.02;
        State setpoint = trapezoidProfile.calculate(t, new State(getArmRad(),toRads(absoluteEncoder.getVelocity())), goal);
        double ff = armFF.calculate(setpoint.position-toRads(17.5), setpoint.velocity);//14 degrees accounts for offset from parallel 
        pidController.setSetpoint(setpoint.position);
        double pidOutput = pidController.calculate(getArmRad());

        motorController.setVoltage(pidOutput+ff);
        motorController2.setVoltage(pidOutput+ff);

        SmartDashboard.putNumber("Arm Rad",getArmRad());
        SmartDashboard.putNumber("armDeg", getArmDeg());
        SmartDashboard.putNumber("armVelDeg", absoluteEncoder.getVelocity());
        SmartDashboard.putNumber("Applied Voltage", pidOutput+ff);
        SmartDashboard.putNumber("Set Pos",setpoint.position);
        SmartDashboard.putNumber("Goal", goal.position);
        SmartDashboard.putNumber("Set Velocity",setpoint.velocity);
        SmartDashboard.putNumber("FF", ff);
        SmartDashboard.putNumber("PID",pidOutput);
        SmartDashboard.putNumber("error",pidController.getPositionError());
        SmartDashboard.putNumber("Setpoint diff",Math.abs(getArmRad()-goal.position) );
    }

    /*
     * Returns true if the arm is within 5 degrees of the passed angle
     * @param Ideal arm state, in radians
     * @param True if the current angle is within 5 degrees of the goal
     */
    public boolean atStatePos(State goal) { // will check if the arm angle is at the setpoint periodically
        if (Math.abs(getArmRad()-goal.position) < toRads(20)) {
            return true;
        } else {
            return false;
        }
    }

    public double toRads(double degrees) {
        return degrees * DEG_TO_RAD;
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

    private Command rotateToIntake() { // sets the set point, angle to intake @ ground
        return new 
        InstantCommand(() -> {
            goal = Arm.intakeState; 
            t=0.0;
                    });
        /*
        .andThen(new WaitUntilCommand(()->this.atStatePos(goal)))
        .andThen(new InstantCommand(()->{
            motorController.setVoltage(0.0);
            motorController2.setVoltage(0.0);
            SmartDashboard.putString("At zero", "true");
        })); */
    }


    public Command rotateToState(State goal){
        if(goal == Arm.intakeState){
            return rotateToIntake();
        }else{
            return new InstantCommand(() -> {this.goal = goal;
            t=0.0;});
        }
    }

    /**
     * calculates the angle to shoot to the speaker at any field pos
     * @param posX the field position of the robot relative to the speaker
     * @return angle (in degrees)
     */
    double calculateAngle(double posX, double posY) { // position relative to subwoofer
        double horDist = Math.sqrt(posX*posX+posY*posY);

        // the angle that the shooter should be
        double shooterAngle = Math.atan(shooterVel + Math.sqrt(Math.pow(shooterVel, 4) - 9.81*(9.81*horDist*horDist + 2*Field.speakerHeight*shooterVel*shooterVel)) / (9.81*horDist));
        
        double armAngle = 180 - Math.toDegrees(shooterAngle) - shooterOffset;
        
        // TODO: consider shooting at an angle a little more because the hood is pretty big
        return armAngle;
    }

    /**
     * calculates the angle to shoot to the speaker at any field pos and rotates the arm
     */
    public InstantCommand rotateToCalculatedAngle(double posX, double posY) {
        calculateAngleState.position = toRads(calculateAngle(posX, posY));

        return new InstantCommand(() -> goal = calculateAngleState); // in degrees
    }
    
}
