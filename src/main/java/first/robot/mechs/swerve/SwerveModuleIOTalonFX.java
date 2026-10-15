package first.robot.mechs.swerve;

import static org.wpilib.units.Units.Amps;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import first.robot.Constants;
import first.robot.Constants.SwerveModuleConfiguration;

public class SwerveModuleIOTalonFX implements SwerveModuleIO {
    private final SwerveModuleConfiguration moduleConfig;

    private final CANBus kCANBus = new CANBus(Constants.CAN_BUS_NAME);
    private final CANcoder cancoder;

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    private final DutyCycleOut driveOut = new DutyCycleOut(0);
    private final DutyCycleOut turnOut = new DutyCycleOut(0);

    private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();

    private final VelocityTorqueCurrentFOC velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final PositionTorqueCurrentFOC positionTorque = new PositionTorqueCurrentFOC(0).withSlot(0);

	private final StatusSignal<Angle> drivePosition, turnPosition, turnAbsPosition;
	private final StatusSignal<AngularVelocity> driveVelocity, turnVelocity;
	private final StatusSignal<Voltage> driveVoltage, turnVoltage;
	private final StatusSignal<Current> driveCurrent, turnCurrent;

	private final Debouncer driveConnectionDebouncer = new Debouncer(Constants.DRIVETRAIN_MOTOR_DISCONNECT_WARNING_THRESHOLD_SECONDS);
	private final Debouncer turnConnectionDebouncer = new Debouncer(Constants.DRIVETRAIN_MOTOR_DISCONNECT_WARNING_THRESHOLD_SECONDS);
	private final Debouncer cancoderConnectionDebouncer = new Debouncer(Constants.DRIVETRAIN_ENCODER_DISCONNECT_WARNING_THRESHOLD_SECONDS);

    public SwerveModuleIOTalonFX(SwerveModuleConfiguration _moduleConfig) {
        moduleConfig = _moduleConfig;

        driveMotor = new TalonFX(moduleConfig.driveMotorID(), kCANBus);
        turnMotor = new TalonFX(moduleConfig.turnMotorID(), kCANBus);
        
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

        cancoder = new CANcoder(moduleConfig.encoderID(), kCANBus);

		drivePosition = driveMotor.getPosition();
		driveVelocity = driveMotor.getVelocity();
		driveVoltage = driveMotor.getMotorVoltage();
		driveCurrent = driveMotor.getStatorCurrent();
		turnPosition = turnMotor.getPosition();
		turnAbsPosition = cancoder.getAbsolutePosition();
		turnVelocity = turnMotor.getVelocity();
		turnVoltage = turnMotor.getMotorVoltage();
		turnCurrent = turnMotor.getStatorCurrent();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
		StatusCode driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveVoltage, driveCurrent);
		StatusCode turnStatus = BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnVoltage, turnCurrent);
		StatusCode cancoderStatus = BaseStatusSignal.refreshAll(turnAbsPosition);

		inputs.driveMotorConnected = driveConnectionDebouncer.calculate(driveStatus.isOK());
		inputs.turnMotorConnected = turnConnectionDebouncer.calculate(turnStatus.isOK());
		inputs.turnAbsEncoderConnected = cancoderConnectionDebouncer.calculate(cancoderStatus.isOK());
		inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
		inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
		inputs.driveAppliedVolts = driveVoltage.getValueAsDouble();
		inputs.driveCurrentSupplyAmps = driveCurrent.getValueAsDouble();
		inputs.turnPositionRad = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
		inputs.turnAbsPositionRad = Rotation2d.fromRotations(turnAbsPosition.getValueAsDouble());
		inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
		inputs.turnAppliedVolts = turnVoltage.getValueAsDouble();
		inputs.turnCurrentSupplyAmps = turnCurrent.getValueAsDouble();
    }

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