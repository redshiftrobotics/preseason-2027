package first.robot.mechs.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public Rotation2d yaw = Rotation2d.ZERO;
        public double angularVelocityRadPerSec;
        public boolean gyroConnected = false;
    }

    default void updateInputs(GyroIOInputs inputs) {}

    default void reset() {}
}
