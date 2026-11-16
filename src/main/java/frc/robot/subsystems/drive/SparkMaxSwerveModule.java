package frc.robot.subsystems.drive;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.Pid;
import frc.robot.Constants.SwerveModulePreset;

/** REV NEO/SparkMax module for the MK4n drive. */
public class SparkMaxSwerveModule implements SwerveModuleIO {
    private static final double kNominalVoltage = 12;
    private static final int kCurrentLimit = 40;

    private final SparkMax driveMotor, turnMotor;
    private final SparkMaxConfig driveConfig, turnConfig;
    private final AnalogInput absoluteEncoder;
    private final PIDController drivePid, turnPid;
    private final double encoderOffsetRad;

    public SparkMaxSwerveModule(SwerveModulePreset preset) {
        driveMotor = new SparkMax(preset.drivePort, MotorType.kBrushless);
        turnMotor = new SparkMax(preset.turnPort, MotorType.kBrushless);

        driveConfig = new SparkMaxConfig();
        driveConfig.voltageCompensation(kNominalVoltage);
        driveConfig.smartCurrentLimit(kCurrentLimit);
        driveConfig.inverted(preset.driveInverted);
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.encoder.positionConversionFactor(preset.driveRotToMeters());
        driveConfig.encoder.velocityConversionFactor(preset.driveRotToMeters() / 60);

        turnConfig = new SparkMaxConfig();
        turnConfig.voltageCompensation(kNominalVoltage);
        turnConfig.smartCurrentLimit(kCurrentLimit);
        turnConfig.inverted(preset.turnInverted);
        turnConfig.idleMode(IdleMode.kBrake);

        applyConfigs(ResetMode.kResetSafeParameters);

        absoluteEncoder = new AnalogInput(preset.encoderPort);
        encoderOffsetRad = preset.encoderOffsetRad;

        drivePid = pid(ModuleConstants.kDrivePid);
        turnPid = pid(ModuleConstants.kTurnPid);
        turnPid.enableContinuousInput(-Math.PI, Math.PI); // steer the short way round
    }

    private static PIDController pid(Pid gains) {
        return new PIDController(gains.p(), gains.i(), gains.d());
    }

    private void applyConfigs(ResetMode resetMode) {
        driveMotor.configure(driveConfig, resetMode, PersistMode.kNoPersistParameters);
        turnMotor.configure(turnConfig, resetMode, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setDriveVelocity(double metersPerSecond) {
        driveMotor.setVoltage(
                clamp(drivePid.calculate(driveMotor.getEncoder().getVelocity(), metersPerSecond)));
    }

    @Override
    public void setTurnAngle(Rotation2d angle) {
        turnMotor.setVoltage(
                clamp(turnPid.calculate(getTurnAngle().getRadians(), angle.getRadians())));
    }

    @Override
    public void setCoast(boolean coast) {
        IdleMode idleMode = coast ? IdleMode.kCoast : IdleMode.kBrake;
        driveConfig.idleMode(idleMode);
        turnConfig.idleMode(idleMode);
        applyConfigs(ResetMode.kNoResetSafeParameters);
    }

    @Override
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    @Override
    public double getDriveVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    @Override
    public double getDrivePositionMeters() {
        return driveMotor.getEncoder().getPosition();
    }

    @Override
    public Rotation2d getTurnAngle() {
        // Analog absolute encoder: 0-5 V spans exactly one wheel rotation.
        double rotations = absoluteEncoder.getAverageVoltage() / RobotController.getVoltage5V();
        return new Rotation2d(rotations * 2 * Math.PI + encoderOffsetRad);
    }

    @Override
    public boolean isConnected() {
        // A SparkMax that is not on the bus reports 0 V for its own supply.
        return driveMotor.getBusVoltage() > 1 && turnMotor.getBusVoltage() > 1;
    }

    private static double clamp(double volts) {
        return MathUtil.clamp(volts, -kNominalVoltage, kNominalVoltage);
    }
}
