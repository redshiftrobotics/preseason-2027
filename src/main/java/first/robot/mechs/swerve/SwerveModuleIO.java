package first.robot.mechs.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        //Drive motor
        public double drivePositionRad;
        public double driveVelocityRadPerSec;
        public boolean driveMotorConnected;
        public double driveAppliedVolts;
        public double driveCurrentSupplyAmps;

        //Turn motor
        public Rotation2d turnPositionRad = Rotation2d.ZERO;
        public Rotation2d turnAbsPositionRad = Rotation2d.ZERO;
        public boolean turnMotorConnected;
        public boolean turnAbsEncoderConnected;
        public double turnVelocityRadPerSec;
        public double turnAppliedVolts;
        public double turnCurrentSupplyAmps;
    }

    default void updateInputs(SwerveModuleIOInputs inputs) {}

    default void setDriveMotorVoltage(double voltage) {}

    default void setTurnMotorVoltage(double voltage) {}

    default void setDriveMotorVelocity(double velocityRadPerSec) {}

    default void setTurnMotorPosition(Rotation2d position) {}

    default void setDriveMotorPidGains(double kp, double ki, double kd) {}

    default void setTurnMotorPidGains(double kp, double ki, double kd) {}

    default void setDriveMotorFFwdGains(double ks, double kv, double ka) {}

    default void setDriveMotorBrake(boolean brake) {}

    default void setTurnMotorBrake(boolean brake) {}

    default void stop() {}
}