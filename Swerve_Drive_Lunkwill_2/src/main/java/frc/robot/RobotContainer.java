package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShootNoteCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.io.File;
import java.util.List;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  private final Field2d m_field = new Field2d();
  private final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick joystick = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  private final ShooterSubsystem shooter = new ShooterSubsystem(Constants.ShooterConstants.indexMotorId,
      Constants.ShooterConstants.shooterMotorId);
  private final SwerveDriveSubsystem drivebase = new SwerveDriveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  public RobotContainer() {
    configureBindings();
    // drivebase.setDefaultCommand(new DriveCommand(
    // drivebase, () -> joystick.getRawAxis(1), () -> joystick.getRawAxis(0), () ->
    // joystick.getRawAxis(2)));

    drivebase.setDefaultCommand(new DriveCommand(
        drivebase,
        () -> driverXbox.getLeftY(),
        () -> driverXbox.getLeftX(),
        () -> driverXbox.getRightX()));

    shooter.setDefaultCommand(new ShootNoteCommand(
        shooter,
        () -> driverXbox.getRightTriggerAxis(),
        () -> driverXbox.getLeftTriggerAxis(),
        () -> driverXbox.getAButton()));
  }

  private void configureBindings() {
    // driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));
    driverXbox.b().onTrue(Commands.runOnce(() -> drivebase.lockWheels()));
  }

  private void setupLogging() {
    DataLogManager.start();

    DriverStation.startDataLog(DataLogManager.getLog());

    SendableRegistry.setSubsystem(drivebase, "DriveSubsystem");
    SmartDashboard.putData(drivebase);
    SmartDashboard.putData("Field", m_field);

    Command periodicLoggingCommand = Commands.run(() -> {
      DataLogManager.log("Robot Pose: " + drivebase.getPose().toString());
      m_field.setRobotPose(drivebase.getPose());
      SmartDashboard.putNumber("Drive/Position/X", drivebase.getPose().getX());
      SmartDashboard.putNumber("Drive/Position/Y", drivebase.getPose().getY());
      SmartDashboard.putNumber("Drive/Rotation", drivebase.getPose().getRotation().getDegrees());
    });

    periodicLoggingCommand.schedule();
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}