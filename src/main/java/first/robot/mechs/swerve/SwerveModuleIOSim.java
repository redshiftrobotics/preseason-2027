package first.robot.mechs.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;
import first.robot.Constants;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private final DCMotor driveMotor = DCMotor.getKrakenX60Foc(1);
    private final DCMotor turnMotor = DCMotor.getKrakenX60Foc(1);

    private double driveVolts = 0.0, driveFFwdVolts = 0.0, turnVolts = 0.0;


    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    private final PIDController drivePID;
    private final PIDController turnPID;

    private final SimpleMotorFeedforward driveFFwd;

    private boolean driveUseClosedLoop = false, turnUseClosedLoop = false;

    public SwerveModuleIOSim() {
        //Don't ask why it's called singleJointedArm... this is just how you do it now... WHYYYYYYYY
        driveSim = new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(driveMotor, 0.025, Constants.DRIVE_REDUCTION), driveMotor);
        turnSim = new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(turnMotor, 0.025, Constants.TURN_REDUCTION), turnMotor);

        //Create controllers
        drivePID = new PIDController(Constants.MODULE_DRIVE_PID_KP, Constants.MODULE_DRIVE_PID_KI, Constants.MODULE_DRIVE_PID_KD);
        turnPID = new PIDController(Constants.MODULE_TURN_PID_KP, Constants.MODULE_TURN_PID_KI, Constants.MODULE_TURN_PID_KD);
        driveFFwd = new SimpleMotorFeedforward(Constants.MODULE_DRIVE_FFWD_KS, Constants.MODULE_DRIVE_FFWD_KV, Constants.MODULE_DRIVE_FFWD_KA);

        //Enable continuous input (wraparound)
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {

    }

    @Override
    public void setDriveMotorVoltage(double voltage) {}

    @Override
    public void setTurnMotorVoltage(double voltage) {}

    @Override
    public void setDriveMotorVelocity(double velocityRadPerSec) {}

    @Override
    public void setTurnMotorPosition(Rotation2d position) {}

    @Override
    public void setDriveMotorPidGains(double kp, double ki, double kd) {}

    @Override
    public void setTurnMotorPidGains(double kp, double ki, double kd) {}

    @Override
    public void setDriveMotorFFwdGains(double ks, double kv, double ka) {}

    @Override
    public void setDriveMotorBrake(boolean brake) {}

    @Override
    public void setTurnMotorBrake(boolean brake) {}

    @Override
    public void stop() {}
}