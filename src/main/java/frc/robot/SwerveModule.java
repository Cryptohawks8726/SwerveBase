package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private int modPos;
    private CANSparkMax driveMotor, steerMotor;
    private CANCoder absEncoder;
    private Transform2d transformationFromCenter;
    private SparkMaxPIDController driveController, steerController;
    private RelativeEncoder driveEncoder, steerEncoder;
    private SwerveModuleState lastSetState;

    public SwerveModule(Constants.Swerve.Module modConstants){
        modPos = modConstants.modPos;
        absEncoder = new CANCoder(modConstants.cancoderid);
        lastSetState = new SwerveModuleState();
        // config can coder

        // turn down status frames on encoder
        transformationFromCenter = modConstants.displacment;

        driveMotor = new CANSparkMax(modConstants.driveMotorid, MotorType.kBrushless);
        steerMotor = new CANSparkMax(modConstants.steerMotorid, MotorType.kBrushless);
        // config SparkMax 
        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();
        // config encoders
        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionFactor); // meters
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionFactor/ 60.0); // m/s
        
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
    }

    public int getModPos(){
        return modPos;
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

    public SwerveModuleState getLastSetState(){
        return lastSetState;
    }
}
