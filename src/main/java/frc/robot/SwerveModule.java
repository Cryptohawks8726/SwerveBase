package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private int modPos;
    private CANSparkMax driveMotor, steerMotor;
    private CANCoder absEncoder;
    private Transform2d transformationFromCenter;
    private SparkMaxPIDController driveController, steerController;
    private RelativeEncoder driveEncoder, steerEncoder;
    private SwerveModuleState lastSetState;
    private SwerveModulePosition simulatedPosition;

    public SwerveModule(Constants.Swerve.Module modConstants){
        modPos = modConstants.modPos;
        absEncoder = new CANCoder(modConstants.canCoderid);
        lastSetState = new SwerveModuleState();
        
        // config can coder
       absEncoder.configMagnetOffset(modConstants.canCoderOffset);
        //absEncoder.setStatusFramePeriod(null, modPos)
        
        transformationFromCenter = modConstants.displacment;

        driveMotor = new CANSparkMax(modConstants.driveMotorid, MotorType.kBrushless);
        steerMotor = new CANSparkMax(modConstants.steerMotorid, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        steerMotor.restoreFactoryDefaults();
        
        // config SparkMax 
        driveMotor.setIdleMode(IdleMode.kCoast);
        steerMotor.setIdleMode(IdleMode.kCoast);
        
        driveMotor.setSmartCurrentLimit(60);
        steerMotor.setSmartCurrentLimit(60);
        // doesn't work for drive, needed for pure rot
        //if(modPos == 1 || modPos == 3){
       //     driveMotor.setInverted(true);
       // }
        
        driveMotor.enableVoltageCompensation(12.0);
        steerMotor.enableVoltageCompensation(12.0);

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();


        // config encoders
        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionFactor); // meters
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionFactor / 60.0); // m/s
        
        steerEncoder.setPositionConversionFactor(360.0 / Constants.Swerve.steerGearRatio); // degrees
        steerEncoder.setVelocityConversionFactor(360.0 / Constants.Swerve.steerGearRatio / 60.0); // d/s

        seedRelativeEncoder();
        

        driveController = driveMotor.getPIDController();
        steerController = steerMotor.getPIDController();
        
        driveController.setP(Constants.Swerve.kDriveP);
        driveController.setI(Constants.Swerve.kDriveI);
        driveController.setD(Constants.Swerve.kDriveD);
        driveController.setFF(Constants.Swerve.kDriveFF);

        steerController.setP(Constants.Swerve.kSteerP);
        steerController.setI(Constants.Swerve.kSteerI);
        steerController.setD(Constants.Swerve.kSteerD);
        steerController.setFF(Constants.Swerve.kSteerFF);

        driveMotor.burnFlash();
        steerMotor.burnFlash();

        // sim setup
        simulatedPosition = new SwerveModulePosition();
    }

    public int getModPos(){
        return modPos;
    }

    public double getAbsPos(){
        return absEncoder.getPosition();
    }
    
    public Transform2d getCenterTransform(){
        return transformationFromCenter;
    }
    
    public void seedRelativeEncoder(){
        steerEncoder.setPosition(absEncoder.getPosition());
    }
    
    public SwerveModule closedLoopDrive(SwerveModuleState setPoint){
        setPoint = SwerveModuleState.optimize(setPoint, Rotation2d.fromDegrees(steerEncoder.getPosition()%360));
        lastSetState = setPoint;
        driveController.setReference(setPoint.speedMetersPerSecond, ControlType.kVelocity); // IDK if velocity control will work well
        steerController.setReference(setPoint.angle.getDegrees(), ControlType.kPosition);
        
        return this;
    }
    
    public void setBrake(){
        driveMotor.setIdleMode(IdleMode.kBrake); // angle motor will always be in brake
    }

    public void setCoast(){
        driveMotor.setIdleMode(IdleMode.kCoast);
    }
    
    public SwerveModuleState getCurrentState(){ // used for odometry
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
}
