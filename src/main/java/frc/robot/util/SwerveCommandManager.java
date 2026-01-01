package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants.OperatorConstants;
import frc.robot.util.Constants.SwerveConstants;
import swervelib.SwerveInputStream;

/**
 * Plain storage class for SwerveSubsystem commands to avoid repetitive code in
 * BindingsBase subclasses
 */
public class SwerveCommandManager {
    // Extra instance of the driver's controller for the swerve
    private static final CommandXboxController johnController = new CommandXboxController(0);

    private final SwerveSubsystem internalSwerve;
    private final SwerveInputStream chassisSpeedsSupplier;
    private final SwerveInputStream slowChassisSpeedsSupplier;
    public final Command driveFieldOriented;
    public final Command slowDriveFieldOriented;
    public final Command driveRobotRelative;

    public SwerveCommandManager(SwerveSubsystem swerve) {
        internalSwerve = swerve;

        // Converts driver input into a field-relative ChassisSpeeds that is controlled
        // by angular velocity.
        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the angular velocity of the robot
        chassisSpeedsSupplier = SwerveInputStream.of(
                swerve.getSwerveDrive(),
                () -> johnController.getLeftY() * -1,
                () -> johnController.getLeftX() * -1)
                .withControllerRotationAxis(() -> -johnController.getRightX())
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(SwerveConstants.defaultTranslationCoefficient)
                .scaleRotation(SwerveConstants.defaultRotationCoefficient)
                .allianceRelativeControl(true);

        // Scales back the chassisspeeds for slow modes
        slowChassisSpeedsSupplier = chassisSpeedsSupplier.copy()
                .scaleTranslation(SwerveConstants.slowTranslationCoefficient)
                .scaleRotation(SwerveConstants.slowRotationCoefficient);

        driveFieldOriented = swerve.driveFieldOriented(chassisSpeedsSupplier).withName("Field Oriented Drive");
        slowDriveFieldOriented = swerve.driveFieldOriented(slowChassisSpeedsSupplier)
                .withName("Slow Field Oriented Drive");
        driveRobotRelative = swerve.drive(() -> {
            return new ChassisSpeeds(
                    chassisSpeedsSupplier.get().vxMetersPerSecond,
                    chassisSpeedsSupplier.get().vyMetersPerSecond,
                    chassisSpeedsSupplier.get().omegaRadiansPerSecond);
        }).withName("Robot Relative Drive");
    }

    // Used for Apriltag and Object Detection alignment
    public Command driveFieldOrientedSpeedOverride(Supplier<ChassisSpeeds> speedSupplier) {
        return internalSwerve.driveFieldOriented(speedSupplier);
    }
}
