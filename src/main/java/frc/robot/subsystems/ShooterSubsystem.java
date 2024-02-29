package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
    
    private final CANSparkMax conveyorMotor = new CANSparkMax(Shooter.conveyorMotorId, MotorType.kBrushless);
    public final CANSparkMax topFlywheelMotor = new CANSparkMax(Shooter.topMotorId, MotorType.kBrushless);
    public final CANSparkMax bottomFlywheelMotor = new CANSparkMax(Shooter.bottomMotorId, MotorType.kBrushless);
    private final RelativeEncoder topFlywheelEncoder = topFlywheelMotor.getEncoder();
    private final RelativeEncoder bottomFlywheelEncoder = bottomFlywheelMotor.getEncoder();

    // Feedforward control
    private double ampSetpoint = 1800;
    private double speakerSetpoint = 5400; // Theoretical, get experimental value top 5570, bottom 5400
    private final double conveyorSetpoint = 12;

    private final double kSTop = 0.0;
    private final double kSBottom = 0.0;

    private final double kVTop = 1.0 / 5570.0;
    private final double kVBottom = 1.0 / 5400.0;
    private final double kVConveyor = 1;

    private double testSetpoint = 0.0;

    // Feedback control
    private final double kP = 0.0001;
    private final double kI = 0;
    private final double kD = 0; // 0.05 works, but causes the motors to tick. This could be resolved by a
                                 // bang-bang controller.;
    private final double kPConveyor = 0;
    private final double kIConveyor = 0;
    private final double kDConveyor = 0;

    SparkPIDController topPID = topFlywheelMotor.getPIDController();
    SparkPIDController bottomPID = bottomFlywheelMotor.getPIDController();
    SparkPIDController conveyorPID = conveyorMotor.getPIDController();

    private DigitalInput beamBreakSensor = new DigitalInput(Shooter.beamBreakReceiverPort); // TODO: Wire the emitter to signal - ground to allow
                                                                // for self-tests of the sensor
    // Configures flywheel motors
    public ShooterSubsystem() {
        topPID.setFF(kVTop);
        bottomPID.setFF(kVBottom);
        
        topPID.setP(kP);
        topPID.setI(kI);
        topPID.setD(kD);
        bottomPID.setP(kP);
        bottomPID.setI(kI);
        bottomPID.setD(kD);
        conveyorPID.setP(kPConveyor);
        conveyorPID.setI(kIConveyor);
        conveyorPID.setD(kDConveyor);

        topFlywheelMotor.setInverted(false);
        bottomFlywheelMotor.setInverted(false);
        conveyorMotor.setInverted(false);

        topFlywheelMotor.setSmartCurrentLimit(40);
        bottomFlywheelMotor.setSmartCurrentLimit(40);
        conveyorMotor.setSmartCurrentLimit(25);

        topFlywheelMotor.enableVoltageCompensation(12.0);
        bottomFlywheelMotor.enableVoltageCompensation(12.0);
        conveyorMotor.enableVoltageCompensation(12.0);        
    };

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top Flywheel Vel", topFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Bottom Flywheel Vel", bottomFlywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Conveyor Vel", conveyorMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Beam Broken", !beamBreakSensor.get());
    }

    /*
     * Returns an InstantCommand to set the flywheel speed reference.
     * Warning: This reference is currently followed by a PID controller, so setting
     * a speed substantially slower than or in the opposite direction
     * of current motion may cause damage to the shooter belts
     * 
     * @param motorSpeed the desired RPM (-5700,5700) acheivable
     * 
     * @return The InstantCommand to set the flywheel reference
     */
    // TODO: Prevent potentially damaging references from being set by checking
    // current flywheel velocity OR rewrite with a bang-bang controller.
    private InstantCommand startFlywheels(double motorSpeed) {
        return new InstantCommand(() -> {
            topPID.setReference(motorSpeed, ControlType.kVelocity, 0, kSTop, ArbFFUnits.kVoltage);
            bottomPID.setReference(motorSpeed, ControlType.kVelocity, 0, kSBottom, ArbFFUnits.kVoltage);
        });
    }

    /*
     * Returns a SequentialCommandGroup that starts the intake conveyor until the
     * beam break sensor is triggered.
     * 
     * @Return The SequentialCommandGroup to run the intake sequence
     */
    public SequentialCommandGroup startIntake() {
        return setConveyorReference(conveyorSetpoint)
                .andThen(setFlywheelReferences(0))
                .andThen(new WaitUntilCommand(() -> isBeamBroken()))
                .andThen(new WaitCommand(0.0425))
                .andThen(setConveyorReference(0));
    }

    public InstantCommand setFlywheelReferences(double newVelocitySetpoint) {
        return new InstantCommand(() -> {
            if (newVelocitySetpoint > 0) {
                topPID.setReference(newVelocitySetpoint, ControlType.kVelocity, 0, kSTop, ArbFFUnits.kVoltage);
                bottomPID.setReference(newVelocitySetpoint, ControlType.kVelocity, 0, kSBottom, ArbFFUnits.kVoltage);
            }
            else {
                topPID.setReference(0, ControlType.kVoltage, 0, kSTop, ArbFFUnits.kVoltage);
                bottomPID.setReference(0, ControlType.kVoltage, 0, kSBottom, ArbFFUnits.kVoltage);
            }
        }, this);
    }

    public InstantCommand setConveyorReference(double newVoltageSetpoint) {
        return new InstantCommand(() -> {
            if (newVoltageSetpoint > 0) {
                conveyorPID.setReference(newVoltageSetpoint, ControlType.kVoltage, 0, conveyorSetpoint, ArbFFUnits.kVoltage);
            }
            else {
                conveyorPID.setReference(0, ControlType.kVoltage, 0, conveyorSetpoint, ArbFFUnits.kVoltage);
            }
        }, this);
    }

    // InstantCommand which activates the index's conveyor belt, transporting a held
    // ring to the flywheel and firing it
    // Only fires if the flywheel is up to speed
    public Command fireNote(boolean isAmp) {
        return startFlywheels(isAmp ? ampSetpoint : speakerSetpoint)//amp was 1000
                .andThen(new WaitUntilCommand(() -> Math.abs(topFlywheelEncoder.getVelocity() - (isAmp ? ampSetpoint : speakerSetpoint)) < 350
                        && Math.abs(bottomFlywheelEncoder.getVelocity() - (isAmp ? ampSetpoint : speakerSetpoint)) < 350)) // Lower tolerance
                                                                                                    // range in the
                                                                                                    // future
                .andThen(new InstantCommand(() -> {
                    conveyorMotor.setSmartCurrentLimit(35);
                    System.out.println(topFlywheelEncoder.getVelocity());
                    System.out.println(bottomFlywheelEncoder.getVelocity());
                }, this))
                .andThen(setConveyorReference(conveyorSetpoint))
                .andThen(new WaitUntilCommand(() -> !isBeamBroken()))
                .andThen(new InstantCommand(() -> {
                    conveyorMotor.setSmartCurrentLimit(25);
                }))
                .andThen(new WaitCommand(0.5))
                .andThen(setFlywheelReferences(0))
                .andThen(setConveyorReference(0))
                .withName("FireNote, isAmp:"+isAmp);
    }

    public Command nudgeIntake(){
        return new InstantCommand(()->conveyorMotor.setVoltage(-5.0))
        .andThen(new WaitCommand(0.1))
        .finallyDo(()->conveyorMotor.setVoltage(0.0));
    }

    public Command staticGainTest() {
        return new InstantCommand(() -> {
            testSetpoint += 0.01;
            SmartDashboard.putNumber("test setpoint", testSetpoint);
            bottomFlywheelMotor.setVoltage(testSetpoint);
        });
    }

    public boolean isBeamBroken() {
        return !beamBreakSensor.get();
    }

    public InstantCommand testCooking() {
        return new InstantCommand(() -> {
            conveyorMotor.setVoltage(12);
        });
    }
}
