package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.SwerveModule;

public class SwerveDrive extends SubsystemBase {
    
    private List<SwerveModule> modules;

    private SwerveModuleState[] modStates;
    private SwerveModulePosition[] modPositions;
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator odometry;
    private ChassisSpeeds lastSetChassisSpeeds;
    private AHRS gyro;
    private AnalogGyroSim simGyro;
    private Matrix<N3,N1> initalVisionStDev;
    //private Matrix<N3,N1> regularVisionStDev;

    private Field2d field; 
    private FieldObject2d modNE,modSE,modSW,modNW;
    private FieldObject2d[] modPoses;
   
    public SwerveDrive(){
        modules = Arrays.asList(
            new SwerveModule(Constants.Swerve.Module.NE),
            new SwerveModule(Constants.Swerve.Module.SE),
            new SwerveModule(Constants.Swerve.Module.SW),
            new SwerveModule(Constants.Swerve.Module.NW)
        );

        modPositions = new SwerveModulePosition[]{
            modules.get(0).getCurrentPosition(),
            modules.get(1).getCurrentPosition(),
            modules.get(2).getCurrentPosition(),
            modules.get(3).getCurrentPosition()
        };

        kinematics = new SwerveDriveKinematics(
            modules.get(0).getCenterTransform().getTranslation(),
            modules.get(1).getCenterTransform().getTranslation(),
            modules.get(2).getCenterTransform().getTranslation(),
            modules.get(3).getCenterTransform().getTranslation()
        );
        
        gyro = new AHRS(SerialPort.Port.kUSB1,SerialDataType.kRawData,(byte) 100);
        gyro.calibrate(); // possibly move to avoid the robot being moved during calibration
        //
        simGyro = new AnalogGyroSim(0);
       
        
        odometry = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), modPositions, new Pose2d()); 
        
        lastSetChassisSpeeds = new ChassisSpeeds();
        
        field = new Field2d();
        modPoses = new FieldObject2d[]{
            field.getObject("modNE"),
            field.getObject("modSE"),
            field.getObject("modSW"),
            field.getObject("modNW")
        };
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic(){
        System.out.print("NE");
        System.out.println(modules.get(0).getAbsPos());
        System.out.print("SE");
        System.out.println(modules.get(1).getAbsPos());
        System.out.print("SW");
        System.out.println(modules.get(2).getAbsPos());
        System.out.print("NW");
        System.out.println(modules.get(3).getAbsPos());
        odometry.update(
            gyro.getRotation2d(), 
            getSwerveModulePositions()
        );
        //zeroModules();
        field.setRobotPose(odometry.getEstimatedPosition());
        
        for (int i = 0;i<4;i++){
            modPoses[i].setPose(
                odometry.getEstimatedPosition()
                .plus(
                    modules.get(i).getCenterTransform()
                    .plus
                        (new Transform2d(new Translation2d(),modules.get(i).getCurrentState().angle))
                    )
            );
        }
       
        //drive(new ChassisSpeeds(0, 0, 0),true);
    }
     
    @Override
    public void simulationPeriodic(){
        
        // multiplying by 0.001 makes it more usable, it isn't based on an accurate time interval
        simGyro.setAngle(simGyro.getAngle() + lastSetChassisSpeeds.omegaRadiansPerSecond*57.2958*0.001);
        odometry.update(
            new Rotation2d(simGyro.getAngle()), 
            getSwerveModulePositions()
        );

        Pose2d estimatedPostition = odometry.getEstimatedPosition();

        field.setRobotPose(estimatedPostition);
        // sim new positions of modules
        for (int i = 0;i<4;i++){
            modPoses[i].setPose(
                estimatedPostition // current robot origin
                .plus(
                    modules.get(i).getCenterTransform() // transform by constant translation to module
                    .plus
                        (new Transform2d(new Translation2d(),modules.get(i).getLastSetState().angle)) // set mod rotation to the last set angle
                    )
            );
        }
        SmartDashboard.putNumber("xpos", estimatedPostition.getTranslation().getX());
        SmartDashboard.putNumber("ypos", estimatedPostition.getTranslation().getY());
        SmartDashboard.putNumber("setXVel", lastSetChassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("setYVel", lastSetChassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("mod0vel", modules.get(0).getLastSetState().speedMetersPerSecond);
        SmartDashboard.putNumber("mod1vel", modules.get(1).getLastSetState().speedMetersPerSecond);
        SmartDashboard.putNumber("mod2vel", modules.get(2).getLastSetState().speedMetersPerSecond);
        SmartDashboard.putNumber("mod3vel", modules.get(3).getLastSetState().speedMetersPerSecond);
        SmartDashboard.putNumber("mod0deg", modules.get(0).getLastSetState().angle.getDegrees());
        SmartDashboard.putNumber("mod1deg", modules.get(1).getLastSetState().angle.getDegrees());
        SmartDashboard.putNumber("mod2deg", modules.get(2).getLastSetState().angle.getDegrees());
        SmartDashboard.putNumber("mod3deg", modules.get(3).getLastSetState().angle.getDegrees());
    };


    public void drive(ChassisSpeeds robotSpeeds, boolean isClosedLoop){  
        lastSetChassisSpeeds = robotSpeeds;
        modStates = kinematics.toSwerveModuleStates(robotSpeeds);
        // SwerveDriveKinematics.desaturateWheelSpeeds(modStates,Constants.Swerve.maxSpeed);
       /* if (robotSpeeds.vxMetersPerSecond == 0.0 && robotSpeeds.vyMetersPerSecond == 0.0 && robotSpeeds.omegaRadiansPerSecond == 0.0){
            
            for (int i = 0; i<4;i++){
                modStates[i] = new SwerveModuleState(modStates[i].speedMetersPerSecond,Rotation2d.fromDegrees(0));
            }
        }*/

       /*if (robotSpeeds.vxMetersPerSecond == 0.0 && robotSpeeds.vyMetersPerSecond == 0.0){
            
            
                modStates[1] = new SwerveModuleState(-modStates[1].speedMetersPerSecond,modStates[1].angle);
                modStates[3] = new SwerveModuleState(-modStates[3].speedMetersPerSecond,modStates[3].angle);

        
        }*/
        modules.forEach(mod -> {mod.closedLoopDrive(modStates[mod.getModPos()]);});
    }

    public StartEndCommand passiveBrake(){
        SwerveModuleState leftToRight = new SwerveModuleState(0.0,Rotation2d.fromDegrees(45));
        SwerveModuleState rightToLeft = new SwerveModuleState(0.0, Rotation2d.fromDegrees(135));
        return new StartEndCommand(
            () -> modules.forEach(mod -> {mod.closedLoopDrive((mod.getModPos() %2 == 0) ? leftToRight : rightToLeft).setBrake();}), 
            () -> modules.forEach(mod -> {mod.setCoast();}), 
            this
        );
    }

    public void zeroModules(){
        modules.get(0).closedLoopDrive(new SwerveModuleState(0,Rotation2d.fromDegrees(Constants.Swerve.Module.NE.canCoderOffset)));
        modules.get(1).closedLoopDrive(new SwerveModuleState(0,Rotation2d.fromDegrees(Constants.Swerve.Module.SE.canCoderOffset)));
        modules.get(2).closedLoopDrive(new SwerveModuleState(0,Rotation2d.fromDegrees(Constants.Swerve.Module.SW.canCoderOffset)));
        modules.get(3).closedLoopDrive(new SwerveModuleState(0,Rotation2d.fromDegrees(Constants.Swerve.Module.NW.canCoderOffset)));

        // modules.forEach(mod -> {mod.closedLoopDrive(new SwerveModuleState(0,Rotation2d.fromDegrees(0)));});
    }

    public Pose2d getPoseEstimate(){
        return odometry.getEstimatedPosition();
    }

    public SwerveModulePosition[] getSwerveModulePositions(){
        
        if (RobotBase.isSimulation()){
           modules.forEach(mod -> {modPositions[mod.getModPos()] = mod.getSimulatedPosition(0.02);});
        } else {
            modules.forEach(mod -> {modPositions[mod.getModPos()] = mod.getCurrentPosition();});
        }
        
        return modPositions;
    }
} 
