package first.robot.mechs.swerve;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import first.robot.Constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOPigeon2 implements GyroIO {

    private final Pigeon2 gyro;

    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;

    private final Debouncer connectionDebouncer = new Debouncer(Constants.DRIVETRAIN_ENCODER_DISCONNECT_WARNING_THRESHOLD_SECONDS);

    public GyroIOPigeon2() {
        gyro = new Pigeon2(Constants.GYRO_DEVICE_ID, new CANBus(Constants.CAN_BUS_NAME));
        yaw = gyro.getYaw();
        yawVelocity = gyro.getAngularVelocityZWorld();

        Pigeon2Configuration config = new Pigeon2Configuration();
        gyro.getConfigurator().apply(config);

        gyro.setYaw(0);
        yaw.setUpdateFrequency(Constants.DRIVETRAIN_ODOMETRY_FREQUENCY_HZ);
        yawVelocity.setUpdateFrequency(Constants.DRIVETRAIN_ODOMETRY_FREQUENCY_HZ);
        gyro.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        StatusCode status = BaseStatusSignal.refreshAll(yaw, yawVelocity);

		inputs.gyroConnected = connectionDebouncer.calculate(status.isOK());
		inputs.yaw = Rotation2d.fromDegrees(yaw.getValueAsDouble());
		inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    }

    public void ResetGyro(){
        gyro.reset();
    }
}