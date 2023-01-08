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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class SwerveDrive extends SubsystemBase {
    
    private List<SwerveModule> modules;

    private SwerveModuleState[] modStates;
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator odometry;
    private AHRS gyro;
    private Matrix<N3,N1> initalVisionStDev;
    private Matrix<N3,N1> regularVisionStDev;

    private Field2d field; 
   
    public SwerveDrive(){
        modules = Arrays.asList(
            new SwerveModule(Constants.Swerve.Module.NE),
            new SwerveModule(Constants.Swerve.Module.SE),
            new SwerveModule(Constants.Swerve.Module.SW),
            new SwerveModule(Constants.Swerve.Module.NW)
        );
        
        kinematics = new SwerveDriveKinematics(
            modules.get(0).getDisplacment(),
            modules.get(1).getDisplacment(),
            modules.get(2).getDisplacment(),
            modules.get(3).getDisplacment()
        );
        
        gyro = new AHRS(SerialPort.Port.kUSB1,SerialDataType.kRawData,(byte) 100);
        gyro.calibrate(); // possibly move to avoid the robot being moved during calibration
        
        // how much we trust vision measurments for odometry
        initalVisionStDev = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.9, 0.9, 0.9); // TODO: lower so the first vision measurement corrects for the wrong initial pose, and then raise again to increase accuracy
        regularVisionStDev = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.9, 0.9, 0.9); 
        odometry = new SwerveDrivePoseEstimator(new Rotation2d(), new Pose2d(), kinematics, new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1), new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02),  initalVisionStDev); // TODO: Update to 2023 Constructor
        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }
 
    @Override
    public void periodic(){
        odometry.update(
            gyro.getRotation2d(), 
            new SwerveModuleState[] {
                modules.get(0).getCurrentState()
                ,modules.get(1).getCurrentState()
                ,modules.get(2).getCurrentState()
                ,modules.get(3).getCurrentState()
            }
        );
        field.setRobotPose(odometry.getEstimatedPosition());
        SmartDashboard.putNumber("NEDeg", modules.get(0).getLastSetState().angle.getDegrees());
        SmartDashboard.putNumber("SEDeg", modules.get(1).getLastSetState().angle.getDegrees());
        SmartDashboard.putNumber("SWDeg", modules.get(2).getLastSetState().angle.getDegrees());
        SmartDashboard.putNumber("NWDeg", modules.get(3).getLastSetState().angle.getDegrees());
    }

    public void drive(ChassisSpeeds robotSpeeds, boolean isClosedLoop){  
        modStates = kinematics.toSwerveModuleStates(robotSpeeds);
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

    public Pose2d getPoseEstimate(){
        return odometry.getEstimatedPosition();
    }
}
