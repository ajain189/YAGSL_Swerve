package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.motors.SparkFlexSwerve;

public class ShooterSubsystem extends SubsystemBase {
    private final VictorSPX indexMotor;
    private final CANSparkFlex shooterMotor;

    public ShooterSubsystem(int indexMotorID, int shooterMotorID) {
        indexMotor = new VictorSPX(indexMotorID);
        shooterMotor = new CANSparkFlex(shooterMotorID, MotorType.kBrushless);
    }

    public void setShooterMotorSpeed(double speed) {
        shooterMotor.set(speed);
    }

    public void setIndexMotorSpeed(double speed) {
        indexMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public void setRobotMotorSpeed(double speed) {
        shooterMotor.set(speed);
        indexMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public void stopIndexMotor() {
        indexMotor.set(VictorSPXControlMode.PercentOutput, 0);
      }
    
      public void stopDriveMotor() {
        shooterMotor.set( 0);
      }
    
      public void stopAllMotors() {
        indexMotor.set(VictorSPXControlMode.PercentOutput, 0);
        shooterMotor.set(0);
      }
}
