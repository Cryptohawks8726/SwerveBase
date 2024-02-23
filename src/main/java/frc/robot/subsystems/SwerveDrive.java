package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.ModulePosition.BL;
import static frc.robot.Constants.Swerve.ModulePosition.BR;
import static frc.robot.Constants.Swerve.ModulePosition.FL;
import static frc.robot.Constants.Swerve.ModulePosition.FR;

import java.util.Arrays;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;


public class SwerveDrive extends SubsystemBase{
    
    public List<SwerveModule> modules;

    private SwerveModuleState[] modStates;
    private SwerveModuleState[] currentModState;
    private SwerveModulePosition[] modPositionStates;
    private SwerveDriveKinematics kinematics;
    public SwerveDrivePoseEstimator odometry;
    public AHRS gyro;
    private DoubleLogEntry log;
    private DoubleLogEntry setModuleAngle0;
    private DoubleLogEntry setModuleAngle1;
    private DoubleLogEntry setModuleAngle2;
    private DoubleLogEntry setModuleAngle3;
    private DoubleLogEntry getModuleAngle0;
    private DoubleLogEntry getModuleAngle1;
    private DoubleLogEntry getModuleAngle2;
    private DoubleLogEntry getModuleAngle3;
    private DoubleLogEntry odometryAngle;
    private DoubleLogEntry getModuleSpeed;
    private DoubleLogEntry setModuleSpeed0;
    private DoubleLogEntry setModuleSpeed1;
    private DoubleLogEntry setModuleSpeed2;
    private DoubleLogEntry setModuleSpeed3;

    private Field2d field; 
    private FieldObject2d[] modPoses;
   
    public SwerveDrive(){
        modules = Arrays.asList(
            new SwerveModule(Constants.Swerve.Module.FR), // 0
            new SwerveModule(Constants.Swerve.Module.BR), // 1
            new SwerveModule(Constants.Swerve.Module.BL), // 2
            new SwerveModule(Constants.Swerve.Module.FL)  // 3
        );

        modPositionStates = new SwerveModulePosition[]{
            modules.get(FR.modPos).getCurrentPosition(),
            modules.get(BR.modPos).getCurrentPosition(),
            modules.get(BL.modPos).getCurrentPosition(),
            modules.get(FL.modPos).getCurrentPosition()
        };

        currentModState = new SwerveModuleState[]{
            modules.get(FR.modPos).getCurrentState(),
            modules.get(BR.modPos).getCurrentState(),
            modules.get(BL.modPos).getCurrentState(),
            modules.get(FL.modPos).getCurrentState()
        };
        
        kinematics = new SwerveDriveKinematics(
            modules.get(FR.modPos).getCenterTransform().getTranslation(),
            modules.get(BR.modPos).getCenterTransform().getTranslation(),
            modules.get(BL.modPos).getCenterTransform().getTranslation(),
            modules.get(FL.modPos).getCenterTransform().getTranslation()
        );
        
        gyro = new AHRS(SerialPort.Port.kUSB1);
        // TODO: check for 2024 version
        //gyro.calibrate(); // possibly move to avoid the robot being moved during calibration        
        // simGyro = new AnalogGyroSim(0);

        DataLogManager.start();

        // Set up custom log entries
        DataLog logs = DataLogManager.getLog();
        /*setModuleAngle0 = new DoubleLogEntry(log, "setModuleAngle0");
        setModuleAngle1 = new DoubleLogEntry(log, "setModuleAngle1");
        setModuleAngle2 = new DoubleLogEntry(log, "setModuleAngle2");
        setModuleAngle3 = new DoubleLogEntry(log, "setModuleAngle3");
        getModuleAngle0 = new DoubleLogEntry(log, "getModuleAngle0");
        getModuleAngle1 = new DoubleLogEntry(log, "getModuleAngle1");
        getModuleAngle2 = new DoubleLogEntry(log, "getModuleAngle2");
        getModuleAngle3 = new DoubleLogEntry(log, "getModuleAngle3");
        odometryAngle = new DoubleLogEntry(log, "odometryAngle");
        getModuleSpeed = new DoubleLogEntry(log, "getModuleSpeed");
        setModuleSpeed0 = new DoubleLogEntry(log, "setModuleSpeed0");
        setModuleSpeed1 = new DoubleLogEntry(log, "setModuleSpeed1");
        setModuleSpeed2 = new DoubleLogEntry(log, "setModuleSpeed2");
        setModuleSpeed3 = new DoubleLogEntry(log, "setModuleSpeed3");*/
        log = new DoubleLogEntry(logs, "loggings");
        
        odometry = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), modPositionStates, new Pose2d()); 
        
        field = new Field2d();
        modPoses = new FieldObject2d[]{
            field.getObject("modFR"),
            field.getObject("modBR"),
            field.getObject("modBL"),
            field.getObject("modFL")
        };
        SmartDashboard.putData("Field", field);
        resetOdometry(new Pose2d());

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
            this::getPoseEstimate, // Robot pose supplier
            this::setOdometryPosition, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5, 0.0, 0), // Translation PID constants
                    new PIDConstants(4, 0, 0), // Rotation PID constants
                    4.5, // Mgax module speed, in m/s
                    Constants.Swerve.driveBaseLength/2, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Blue;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    @Override
    public void periodic(){
        odometry.update(
            gyro.getRotation2d(), 
            getSwerveModulePositions()
        );

        modules.forEach(mod->{mod.updateSteerPid();}); 
        // show estimated robot and mod poses on dashboard
        field.setRobotPose(odometry.getEstimatedPosition());
    
        /*for (int i = 0;i<4;i++){
            modPoses[i].setPose(
                odometry.getEstimatedPosition()
                .plus(
                    modules.get(i).getCenterTransform()
                    .plus
                        (new Transform2d(new Translation2d(),modules.get(i).getCurrentState().angle))
                    )
            );
        }*/
        
        logValues(true);

        
        SmartDashboard.putNumber("estimatedRobotAngle", getRobotAngle().getDegrees());
        SmartDashboard.putNumber("module0Angle", modules.get(0).getAngle());
        SmartDashboard.putNumber("module1Angle", modules.get(1).getAngle());
        SmartDashboard.putNumber("module2Angle", modules.get(2).getAngle());
        SmartDashboard.putNumber("module3Angle", modules.get(3).getAngle());
        SmartDashboard.putNumber("module0EstAngle", modules.get(0).getSetStateAngle());
        SmartDashboard.putNumber("module1EstAngle", modules.get(1).getSetStateAngle());
        SmartDashboard.putNumber("module2EstAngle", modules.get(2).getSetStateAngle());
        SmartDashboard.putNumber("module3EstAngle", modules.get(3).getSetStateAngle());
        SmartDashboard.putNumber("module0RelVel", modules.get(0).getRelativeVel());
        SmartDashboard.putNumber("module1RelVel", modules.get(1).getRelativeVel());
        SmartDashboard.putNumber("module2RelVel", modules.get(2).getRelativeVel());
        SmartDashboard.putNumber("module3RelVel", modules.get(3).getRelativeVel());
        SmartDashboard.putNumber("relativeSpeed", getRobotRelativeSpeeds().omegaRadiansPerSecond);
        SmartDashboard.putNumber("actualRobotAngle", -gyro.getAngle());//gyro.getRotation2d().getDegrees()
        SmartDashboard.putNumber("angleError", gyro.getAngle() + 180);

    }
    /* 
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

    };*/

    
    public void drive(ChassisSpeeds robotSpeeds, boolean isClosedLoop){  
        robotSpeeds = ChassisSpeeds.discretize(robotSpeeds, 0.2);
        modStates = kinematics.toSwerveModuleStates(robotSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(modStates,Constants.Swerve.maxSpeed);
        //SmartDashboard.putNumber("expectedRotation", robotSpeeds.omegaRadiansPerSecond);
        //SmartDashboard.putNumber("actualRotation", getRobotRelativeSpeeds().omegaRadiansPerSecond);
        //SmartDashboard.putNumber("chassisX", robotSpeeds.vxMetersPerSecond);
        //SmartDashboard.putNumber("actualX", getRobotRelativeSpeeds().vxMetersPerSecond);
        double endPointX = 9.88353157043457; //in meters
        double startPointX = 2.889869213104248;
        
        double endPointY = 7.009701728820801;
        double startPointY = 7.009701728820801;



        //SmartDashboard.putNumber("OdometryAngle", ));
        /*SmartDashboard.putNumber("Gyro angle:", getRobotAngle().getDegrees()%360);
        SmartDashboard.putNumber("AngleModule0", modules.get(0).getAngle());
        SmartDashboard.putNumber("AngleModule1", modules.get(1).getAngle());
        SmartDashboard.putNumber("AngleModule2", modules.get(2).getAngle());
        SmartDashboard.putNumber("AngleModule3", modules.get(3).getAngle());
        SmartDashboard.putNumber("setAngleModule0", modules.get(0).getSetStateAngle());
        SmartDashboard.putNumber("setAngleModule1", modules.get(1).getSetStateAngle());
        SmartDashboard.putNumber("setAngleModule2", modules.get(2).getSetStateAngle());
        SmartDashboard.putNumber("setAngleModule3", modules.get(3).getSetStateAngle());
        

        SmartDashboard.putNumber("estimatedModuleSpeed", getRobotRelativeSpeeds().omegaRadiansPerSecond);
        SmartDashboard.putNumber("moduleSpeed0", modules.get(0).getRelativeVel());
        SmartDashboard.putNumber("moduleSpeed1", modules.get(1).getRelativeVel());
        SmartDashboard.putNumber("moduleSpeed2", modules.get(2).getRelativeVel());
        SmartDashboard.putNumber("moduleSpeed3", modules.get(3).getRelativeVel());*/


        /*odometryAngle.append(odometry.getEstimatedPosition().getRotation().getDegrees());
        getModuleAngle0.append(modules.get(0).getAngle());
        getModuleAngle1.append(modules.get(1).getAngle());
        getModuleAngle2.append(modules.get(2).getAngle());
        getModuleAngle3.append(modules.get(3).getAngle());
        setModuleAngle0.append(modules.get(0).getSetStateAngle());
        setModuleAngle1.append(modules.get(1).getSetStateAngle());
        setModuleAngle2.append(modules.get(2).getSetStateAngle());
        setModuleAngle3.append(modules.get(3).getSetStateAngle());
        setModuleSpeed0.append(modules.get(0).getRelativeVel());
        setModuleSpeed1.append(modules.get(1).getRelativeVel());
        setModuleSpeed2.append(modules.get(2).getRelativeVel());
        setModuleSpeed3.append(modules.get(3).getRelativeVel());
        getModuleSpeed.append(getRobotRelativeSpeeds().omegaRadiansPerSecond);*/

        //SmartDashboard.putNumber("supposedDistance", endPointX - startPointX);
        //SmartDashboard.putNumber("rotationPosition", gyro.getAngle());
        //SmartDashboard.putNumber("expectedGyroAngle", odometry.getEstimatedPosition().getRotation().getDegrees());
        //SmartDashboard.putNumber("relativeSpeed", getRobotRelativeSpeeds().omegaRadiansPerSecond);
        //SmartDashboard.putNumber("estimatedPositionX", endPointX - odometry.getEstimatedPosition().getX());
        //SmartDashboard.putNumber("actualAngleError",  180 - getRobotAngle().getDegrees());        
        //SmartDashboard.putNumber("estimatedAngleError",  180 - odometry.getEstimatedPosition().getRotation().getDegrees());
        //SmartDashboard.putNumber("estimatedPositionY", 7.009701728820801 - odometry.getEstimatedPosition().getY());


        SmartDashboard.putNumber("", 0);
        if (isClosedLoop){
            modules.forEach(mod -> {mod.closedLoopDrive(modStates[mod.getModPos().getVal()]);});
        } 
        else if(!isClosedLoop){
            modules.forEach(mod -> {mod.openLoopDrive(modStates[mod.getModPos().getVal()]);});
        }
        
    }

    
    public void drive(ChassisSpeeds robotSpeeds){
        drive(robotSpeeds,false);
        
    }

    public StartEndCommand passiveBrake(){
        SwerveModuleState leftToRight = new SwerveModuleState(0.0,Rotation2d.fromDegrees(45));
        SwerveModuleState rightToLeft = new SwerveModuleState(0.0, Rotation2d.fromDegrees(135));
        return new StartEndCommand(
            () -> modules.forEach(mod -> {mod.closedLoopDrive((mod.getModPos().getVal() %2 == 0) ? leftToRight : rightToLeft).setBrake();}), 
            () -> modules.forEach(mod -> {mod.setCoast();}), 
            this
        );
    }

    public AHRS getGyro(){
        return gyro;
    }

    public void normalZeroModules(){
        modules.forEach(mod -> {mod.closedLoopDrive(new SwerveModuleState(0,Rotation2d.fromDegrees(0)));});
    }

    public Pose2d getPoseEstimate(){
        return odometry.getEstimatedPosition();
    }

    public void setOdometryPosition(Pose2d setPosition){
        odometry.resetPosition(getRobotAngle(), getSwerveModulePositions(), setPosition);
        gyro.setAngleAdjustment(setPosition.getRotation().getDegrees()-(gyro.getRotation2d().getDegrees()%360));
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(0.0), getSwerveModulePositions(), pose);
        //SmartDashboard.putNumber("resetOdometryAngle",gyro.getRotation2d().getDegrees()%360);

        gyro.setAngleAdjustment(gyro.getRotation2d().getDegrees()%360);
 
        //gyro.reset();
    }

    public Rotation2d getRobotAngle() {
        // return Rotation2d.fromDegrees(gyro.getYaw());
        return gyro.getRotation2d();
    }

    public SwerveModulePosition[] getSwerveModulePositions(){
        modules.forEach(mod -> {modPositionStates[mod.getModPos().getVal()] = mod.getCurrentPosition();});
        return modPositionStates;
    }

    public SwerveModuleState[] getSwerveModuleStates(){
        modules.forEach(mod -> {currentModState[mod.getModPos().getVal()] = mod.getCurrentState();});
        
        return currentModState;
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public void setModuleStates(SwerveModuleState[] updatedstates){
        SwerveDriveKinematics.desaturateWheelSpeeds(updatedstates, Constants.Swerve.maxSpeed);
        modules.forEach(mod -> {mod.closedLoopDrive(updatedstates[mod.getModPos().getVal()]);});
    }

    public void setEncoderOffsets(){
        modules.forEach(mod -> {mod.setEncoderOffset();});
    }

    public InstantCommand resetGyroAngle(){
        return new InstantCommand(() -> resetOdometry(new Pose2d()));
    }

    public void logValues(boolean moduleLevel){ 
        Pose2d estimatedPostition = odometry.getEstimatedPosition();


        

        //SmartDashboard.putNumber("xpos", estimatedPostition.getTranslation().getX());
        //SmartDashboard.putNumber("ypos", estimatedPostition.getTranslation().getY());
        //SmartDashboard.putNumber("estimatedthetaPos",estimatedPostition.getRotation().getDegrees());
        //SmartDashboard.putNumber("gyroAngle", gyro.getRotation2d().getDegrees());//getRotation2d().getDegrees()%360
        //SmartDashboard.putBoolean("isGyroConnected", gyro.isConnected());
        //SmartDashboard.putNumber("CalcThetaVel", getRobotRelativeSpeeds().omegaRadiansPerSecond);
        //SmartDashboard.putNumber("e", 0);
       // SmartDashboard.putNumber("setXVel", lastSetChassisSpeeds.vxMetersPerSecond);
        //SmartDashboard.putNumber("setYVel", lastSetChassisSpeeds.vyMetersPerSecond);
        if(moduleLevel){
            for (SwerveModule module : modules) {
                String modName = module.getModPos().toString();
                module.seedRelativeEncoder();
                //SmartDashboard.putNumber(modName + "setvel", module.getLastSetState().speedMetersPerSecond);
                //SmartDashboard.putNumber(modName + "actvel", module.getCurrentState().speedMetersPerSecond);
                //SmartDashboard.putNumber(modName + "setdeg", module.getSetStateAngle());
                //SmartDashboard.putNumber(modName + "actdeg", module.getCurrentState().angle.getDegrees());
                
            // SmartDashboard.putNumber(modName + "absdeg", module.getAbsPos());
            //  SmartDashboard.putNumber(modName + "built in steer", module.getRelativePos());
            // SmartDashboard.putNumber(modName + "built in drive", module.getRelativeVel());
                //SmartDashboard.putNumber(modName + "drive current", module.getDriveCurrent());
                //SmartDashboard.putNumber(modName + "steer current", module.getSteerCurrent());
                
            }
        }
        
    }
}