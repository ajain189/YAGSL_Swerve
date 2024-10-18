package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends Command{
    private final SwerveDriveSubsystem swerve;
    private final DoubleSupplier translationX;
    private final DoubleSupplier translationY;
    private final DoubleSupplier rotation;
    private final SlewRateLimiter translationXSlewRateLimiter, translationYSlewRateLimiter, rotationSlewRateLimiter;

    public DriveCommand(SwerveDriveSubsystem swerve, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
        this.swerve = swerve;
        this.translationX = translationX;
        this.translationY = translationY;
        this.rotation = rotation;

        this.translationXSlewRateLimiter = new SlewRateLimiter(8);
        this.translationYSlewRateLimiter = new SlewRateLimiter(8);
        this.rotationSlewRateLimiter = new SlewRateLimiter(8);

        addRequirements(swerve);
    }

    public void execute() {
        double xInput = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.DRIVE_DEADBAND);
        double yInput = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.DRIVE_DEADBAND);
        double rotInput = MathUtil.applyDeadband(rotation.getAsDouble(), Constants.DRIVE_DEADBAND);

        xInput = translationXSlewRateLimiter.calculate(xInput);
        yInput =translationYSlewRateLimiter.calculate(yInput);
        rotInput = rotationSlewRateLimiter.calculate(rotInput);

        Translation2d translation = new Translation2d(xInput, yInput).times(Constants.MAX_SPEED);

        swerve.drive(translation, rotInput * Constants.MAX_ANGULAR_VELOCITY, true);
    }

    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, true);
    }
}
