package frc.robot.subsystems.drive.SwerveModule;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.SwerveModulePreset;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private final SparkMax driveMotor, turnMotor;
    private final SparkMaxConfig driveConfig, turnConfig;
    private final AnalogInput absoluteEncoder;
    private final PIDController drivePID, turnPID;
    private final double encoderOffset;

    public SwerveModuleIOSparkMax(SwerveModulePreset preset) {
        driveMotor = new SparkMax(preset.drivePort, MotorType.kBrushless);
        turnMotor = new SparkMax(preset.turnPort, MotorType.kBrushless);

        driveConfig = new SparkMaxConfig();
        turnConfig = new SparkMaxConfig();

        driveConfig.voltageCompensation(12.0);
        driveConfig.smartCurrentLimit(40);
        driveConfig.inverted(preset.driveInverted);
        driveConfig.idleMode(IdleMode.kBrake);

        driveConfig.encoder.positionConversionFactor(preset.driveRot2Meters());
        driveConfig.encoder.velocityConversionFactor(preset.driveRPM2MetersPerSec());

        turnConfig.voltageCompensation(12.0);
        turnConfig.smartCurrentLimit(40);
        turnConfig.inverted(preset.turnInverted);
        turnConfig.idleMode(IdleMode.kBrake);

        turnConfig.encoder.positionConversionFactor(preset.turnRot2Rads());
        turnConfig.encoder.velocityConversionFactor(preset.turnRPM2RadsPerSec());

        absoluteEncoder = new AnalogInput(preset.encoderPort);
        encoderOffset = preset.encoderOffset;

        turnPID =
                new PIDController(
                        SwerveModuleConstants.kSparkMaxPID.turnP,
                        SwerveModuleConstants.kSparkMaxPID.turnI,
                        SwerveModuleConstants.kSparkMaxPID.turnD);
        drivePID =
                new PIDController(
                        SwerveModuleConstants.kSparkMaxPID.driveP,
                        SwerveModuleConstants.kSparkMaxPID.driveI,
                        SwerveModuleConstants.kSparkMaxPID.driveD);
    }

    public void setVelocity(double velocity) {
        driveMotor.setVoltage(drivePID.calculate(getVelocity(), velocity));
    }

    public void setRotation(Rotation2d rotation) {
        SwerveModuleState state = new SwerveModuleState(getState().speedMetersPerSecond, rotation);
        state.optimize(getState().angle);
        turnMotor.setVoltage(turnPID.calculate(getRotation().getRadians(), rotation.getRadians()));
    }

    public void stop() {
        turnMotor.set(0);
        driveMotor.set(0);
    }

    public double getVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    public Rotation2d getRotation() {
        double angle = absoluteEncoder.getAverageVoltage() / RobotController.getVoltage5V();
        angle = Units.rotationsToRadians(angle);
        angle += encoderOffset;
        angle = MathUtil.inputModulus(angle, -Math.PI, Math.PI);
        return new Rotation2d(angle);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getRotation());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getEncoder().getPosition(), getRotation());
    }
}
