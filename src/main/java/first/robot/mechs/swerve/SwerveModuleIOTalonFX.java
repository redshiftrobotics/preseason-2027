package first.robot.mechs.swerve;

import static org.wpilib.units.Units.Amps;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Rotation2d;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import first.robot.Constants;
import first.robot.Constants.SwerveModuleConfiguration;

public class SwerveModuleIOTalonFX implements SwerveModuleIO {
    private final SwerveModuleConfiguration moduleConfig;

    private final CANBus kCANBus = new CANBus(Constants.CAN_BUS_NAME);

    private final TalonFX driveMotor = new TalonFX(0, kCANBus);
    private final TalonFX turnMotor = new TalonFX(0, kCANBus);

    private final DutyCycleOut driveOut = new DutyCycleOut(0);
    private final DutyCycleOut turnOut = new DutyCycleOut(0);

    private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();

    private final VelocityTorqueCurrentFOC velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final PositionTorqueCurrentFOC positionTorque = new PositionTorqueCurrentFOC(0).withSlot(0);

    public SwerveModuleIOTalonFX(SwerveModuleConfiguration _moduleConfig) {
        moduleConfig = _moduleConfig;
        
        driveConfig.Slot0.kS = 2.5;
        driveConfig.Slot0.kP = 5;
        driveConfig.Slot0.kI = 0;
        driveConfig.Slot0.kD = 0;

        driveConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40)).withPeakReverseTorqueCurrent(Amps.of(-40));
        driveMotor.getConfigurator().apply(driveConfig);

        turnConfig.Slot0.kP = 60;
        turnConfig.Slot0.kI = 0;
        turnConfig.Slot0.kD = 6;

        driveConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120)).withPeakReverseTorqueCurrent(Amps.of(-120));
        turnMotor.getConfigurator().apply(driveConfig);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {}

    @Override
    public void setDriveMotorVoltage(double voltage) {
        driveOut.Output = voltage;
        driveMotor.setControl(driveOut);
    }

    @Override
    public void setTurnMotorVoltage(double voltage) {
        turnOut.Output = voltage;
        turnMotor.setControl(turnOut);
    }

    @Override
    public void setDriveMotorVelocity(double velocityRadPerSec) {
        // Division is because there's no method Unit to convert radians per second to rotations per second
        driveMotor.setControl(velocityTorque.withVelocity(velocityRadPerSec / (2*Math.PI)));
    }

    @Override
    public void setTurnMotorPosition(Rotation2d position) {
        turnMotor.setControl(positionTorque.withPosition(position.getRotations()));
    }

    @Override
    public void setDriveMotorPidGains(double kp, double ki, double kd) {
        driveConfig.Slot0.kP = kp;
        driveConfig.Slot0.kI = ki;
        driveConfig.Slot0.kD = kd;

        driveMotor.getConfigurator().apply(driveConfig);
    }

    @Override
    public void setTurnMotorPidGains(double kp, double ki, double kd) {
        turnConfig.Slot0.kP = kp;
        turnConfig.Slot0.kI = ki;
        turnConfig.Slot0.kD = kd;

        turnMotor.getConfigurator().apply(turnConfig);
    }

    @Override
    public void setDriveMotorFFwdGains(double ks, double kv, double ka) {
        driveConfig.Slot0.kS = ks;
        driveConfig.Slot0.kV = kv;
        driveConfig.Slot0.kA = ka;

        driveMotor.getConfigurator().apply(driveConfig);
    }

    @Override
    public void setDriveMotorBrake(boolean brake) {
        driveConfig.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveConfig);
    }

    @Override
    public void setTurnMotorBrake(boolean brake) {
        turnConfig.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        turnMotor.getConfigurator().apply(turnConfig);
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }
}