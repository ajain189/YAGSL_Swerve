package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;
    

    public SwerveDriveSubsystem(File directory) {
        try {
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
            swerveDrive = new SwerveParser(directory).createSwerveDrive(4.6);
            swerveDrive.setHeadingCorrection(false);
            swerveDrive.setCosineCompensator(false);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.resetDriveEncoders();
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(() -> {
        drive(
            new Translation2d(
                translationX.getAsDouble() * Constants.MAX_SPEED,
                translationY.getAsDouble() * Constants.MAX_SPEED
            ),
            rotation.getAsDouble() * Constants.MAX_ANGULAR_VELOCITY,
            true
        );
    });
}

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public void lockWheels() {
        swerveDrive.lockPose();
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
    }
} 