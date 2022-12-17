package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class SwerveDrive extends SubsystemBase {
    private SwerveModule[] modules = {
        new SwerveModule(Constants.Swerve.Module.NE),
        new SwerveModule(Constants.Swerve.Module.SE),
        new SwerveModule(Constants.Swerve.Module.SW),
        new SwerveModule(Constants.Swerve.Module.NW)
    };
    private SwerveModuleState[] modStates;
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        modules[0].getDisplacment(),
        modules[1].getDisplacment(),
        modules[2].getDisplacment(),
        modules[3].getDisplacment()
    );
    
    public SwerveDrive(){
        
    }

    public void drive(ChassisSpeeds robotSpeeds){ 
        modStates = kinematics.toSwerveModuleStates(robotSpeeds);
        Arrays.asList(modules).forEach(mod -> {mod.drive(modStates[mod.getModPos()]);});
    }
}
