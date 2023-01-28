package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Swerve.ModulePosition;
import io.github.oblarg.oblog.Loggable;

public class SwerveModule implements Loggable{
    private ModulePosition modPos;
    private CANSparkMax driveMotor, steerMotor;
    private CANCoder absEncoder;
    private double canCoderOffset;
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
        canCoderOffset = modConstants.canCoderOffset;
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
        steerMotor.setSmartCurrentLimit(40);
        
        // doesn't work for drive, needed for pure rot
        //if(modPos == 1 || modPos == 3){
       //     driveMotor.setInverted(true);
       // }
        if(modPos.equals(ModulePosition.BL)){
            driveMotor.setInverted(true);
        }
        
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
        steerController = steerMotor.getPIDController();
        
        driveController.setP(Constants.Swerve.kDriveP);
        driveController.setI(Constants.Swerve.kDriveI);
        driveController.setD(Constants.Swerve.kDriveD);
        driveController.setFF(Constants.Swerve.kDriveFF);

        steerController.setP(Constants.Swerve.kSteerP);
        steerController.setI(Constants.Swerve.kSteerI);
        steerController.setD(Constants.Swerve.kSteerD);
        steerController.setFF(Constants.Swerve.kSteerFF);


        if (modPos.equals(ModulePosition.FL)|| modPos.equals(ModulePosition.FR)){
            driveMotor.setInverted(true);
        }
        driveMotor.burnFlash();
        steerMotor.burnFlash();

        // sim setup
        simulatedPosition = new SwerveModulePosition();
        
        setEncoderOffset();
        // sketchy delay to make sure cancoder offsets are saved
        double finishTime = System.currentTimeMillis() + 200;
        while (System.currentTimeMillis() < finishTime) {}
        seedRelativeEncoder();
    }

    public ModulePosition getModPos(){
        return modPos;
    }

    public double getAbsPos(){
        return absEncoder.getPosition();
    }
    
    public double getRelativePos(){
        return steerEncoder.getPosition();
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
        steerEncoder.setPosition(absEncoder.getPosition());
    }
    
    public SwerveModule closedLoopDrive(SwerveModuleState setPoint){
        // setPoint = SwerveModuleState.optimize(setPoint, Rotation2d.fromDegrees(steerEncoder.getPosition()));
        lastSetState = setPoint;
        driveController.setReference(setPoint.speedMetersPerSecond, ControlType.kVelocity); // IDK if velocity control will work well
        System.out.println(setPoint.angle.getDegrees()%180);
        steerController.setReference(setPoint.angle.getDegrees()%180, ControlType.kPosition);
        System.out.println(setPoint.angle.getDegrees()%180);
        
        return this;
    }
    
    public void setBrake(){
        driveMotor.setIdleMode(IdleMode.kBrake); // angle motor will always be in brake
    }

    public void setCoast(){
        driveMotor.setIdleMode(IdleMode.kCoast);
    }
    
    public void stop(){
        driveController.setReference(0, ControlType.kVelocity); // IDK if velocity control will work well
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

    
    public double getLastSetStateAngle(){
        return lastSetState.angle.getDegrees();
    }

}
