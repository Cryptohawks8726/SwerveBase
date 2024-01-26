package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Swerve.ModulePosition;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class SwerveModule implements Loggable{
    private ModulePosition modPos;
    private CANSparkMax driveMotor, steerMotor;
    private CANCoder absEncoder;
    private double canCoderOffset;
    private Transform2d transformationFromCenter;
    private SparkMaxPIDController driveController;
    private RelativeEncoder driveEncoder, steerEncoder;
    private SwerveModuleState lastSetState;
    private SwerveModulePosition simulatedPosition;
    private PIDController contSteerController;
    private SimpleMotorFeedforward velFF;

    public SwerveModule(Constants.Swerve.Module modConstants){
        modPos = modConstants.modPos;
        lastSetState = new SwerveModuleState();
        
        absEncoder = new CANCoder(modConstants.canCoderid);
        absEncoder.configFactoryDefault();
        canCoderOffset = modConstants.canCoderOffset;
        absEncoder.setPositionToAbsolute(0);
        absEncoder.configMagnetOffset(canCoderOffset);        
        //absEncoder.setStatusFramePeriod(null, modPos)
        
        transformationFromCenter = modConstants.displacment;

        driveMotor = new CANSparkMax(modConstants.driveMotorid, MotorType.kBrushless);
        steerMotor = new CANSparkMax(modConstants.steerMotorid, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        steerMotor.restoreFactoryDefaults();
        
        driveMotor.setIdleMode(IdleMode.kCoast);
        steerMotor.setIdleMode(IdleMode.kCoast);
        
        //lower later
        driveMotor.setSmartCurrentLimit(40); 
        steerMotor.setSmartCurrentLimit(30);
        
        driveMotor.enableVoltageCompensation(12.0);
        steerMotor.enableVoltageCompensation(12.0);

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        // config encoders
        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionFactor); // meters
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionFactor / 60.0); // m/s
        
        steerEncoder.setPositionConversionFactor(360.0 / Constants.Swerve.steerGearRatio); // degrees
        steerEncoder.setVelocityConversionFactor(360.0 / Constants.Swerve.steerGearRatio / 60.0); // d/s

        driveController = driveMotor.getPIDController();
        velFF = new SimpleMotorFeedforward(0.0, 2.4);
        contSteerController = new PIDController(Constants.Swerve.kSteerP, Constants.Swerve.kSteerI, Constants.Swerve.kSteerD);
        contSteerController.enableContinuousInput(0, 360);

        driveController.setP(Constants.Swerve.kDriveP);
        driveController.setI(Constants.Swerve.kDriveI);
        driveController.setD(Constants.Swerve.kDriveD);
        driveController.setFF(Constants.Swerve.kDriveFF);       
        driveMotor.setInverted(true);

        driveMotor.burnFlash();
        steerMotor.burnFlash();
        
        // sim setup
        simulatedPosition = new SwerveModulePosition();

        // sketchy delay to make sure cancoder offsets are saved
        double finishTime = System.currentTimeMillis() + 200;
        while (System.currentTimeMillis() < finishTime) {}
        seedRelativeEncoder();
        
    }

    public ModulePosition getModPos(){
        return modPos;
    }

    public double getAbsPos(){
        return absEncoder.getAbsolutePosition();
    }
    
    public double getRelativePos(){
        return steerEncoder.getPosition();
    }

    public double getRelativeVel() {
        return driveEncoder.getVelocity();
    }

    public double getSteerCurrent() {
        return steerMotor.getOutputCurrent();
    }

    public double getDriveCurrent() {
        return driveMotor.getOutputCurrent();
    }

    public Transform2d getCenterTransform(){
        return transformationFromCenter;
    }

    public void setEncoderOffset(){
        absEncoder.configMagnetOffset(canCoderOffset);
        /*if(absEncoder.configMagnetOffset(canCoderOffset).value != 0){
            System.out.println("CanCoder offset error for mod"+modPos.toString());
        }*/
    }

    public void seedRelativeEncoder(){
        steerEncoder.setPosition(absEncoder.getAbsolutePosition());
    }
    
    public SwerveModule closedLoopDrive(SwerveModuleState setPoint){
        SwerveModuleState newSetPoint = SwerveModuleState.optimize(setPoint, Rotation2d.fromDegrees(absEncoder.getAbsolutePosition()));
        lastSetState = newSetPoint;
        driveController.setReference(newSetPoint.speedMetersPerSecond, ControlType.kVelocity,0,velFF.calculate(newSetPoint.speedMetersPerSecond),ArbFFUnits.kVoltage); // IDK if velocity control will work well
        steerMotor.set(contSteerController.calculate(absEncoder.getAbsolutePosition(), MathUtil.inputModulus(newSetPoint.angle.getDegrees(), 0, 360)));
        
        return this;
    }
    
    public void setBrake(){
        driveMotor.setIdleMode(IdleMode.kBrake); 
    }

    public void setCoast(){
        driveMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setPercentOutput(double output){
        driveMotor.set(output);
    }
    
    public void updateSteerPid(){
        steerMotor.set(contSteerController.calculate(absEncoder.getAbsolutePosition()));
    }
    
    public SwerveModuleState getCurrentState(){ 
        return new SwerveModuleState(driveEncoder.getVelocity(),Rotation2d.fromDegrees(steerEncoder.getPosition()%360));
    }
    
    public SwerveModulePosition getCurrentPosition(){ // used for odometry
        return new SwerveModulePosition(driveEncoder.getPosition(),Rotation2d.fromDegrees(steerEncoder.getPosition()%360));
    }

    public SwerveModulePosition getSimulatedPosition(double timeStep){
        double newPosition = simulatedPosition.distanceMeters + (lastSetState.speedMetersPerSecond*timeStep);
        simulatedPosition = new SwerveModulePosition(newPosition, lastSetState.angle);
        return simulatedPosition;
    }
    
    public SwerveModuleState getLastSetState(){
        return lastSetState;
    }
    
    public double getLastSetStateSpeed(){
        return lastSetState.speedMetersPerSecond;
    }

    public double getSetStateAngle(){
        return MathUtil.inputModulus(lastSetState.angle.getDegrees(), 0, 360);
    }

}