package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

/** Every tunable number in one place. Units are meters, radians and seconds unless noted. */
public final class Constants {
    private Constants() {}

    public static final class OIConstants {
        public static final int kDriverPort = 0;

        /** Stick deadband in 0-1 units; inputs inside it are treated as zero. */
        public static final double kDeadband = 0.11;
    }

    public static final class CAN {
        public static final int kDriveMotorPortFL = 10;
        public static final int kTurnMotorPortFL = 20;

        public static final int kDriveMotorPortFR = 11;
        public static final int kTurnMotorPortFR = 21;

        public static final int kDriveMotorPortBL = 12;
        public static final int kTurnMotorPortBL = 22;

        public static final int kDriveMotorPortBR = 13;
        public static final int kTurnMotorPortBR = 23;

        public static final int kPigeonPort = 30;
    }

    /** PID gains for a swerve module; shared by every module implementation. */
    public record Pid(double p, double i, double d) {}

    /** One swerve module's wiring plus the gearing that turns motor rotations into meters. */
    public static final class SwerveModulePreset {
        public final int drivePort, turnPort, encoderPort;
        public final double encoderOffsetRad;
        public final boolean driveInverted, turnInverted;
        private final double driveRatio;

        public SwerveModulePreset(
                int drivePort,
                int turnPort,
                boolean driveInverted,
                boolean turnInverted,
                int encoderPort,
                double encoderOffsetRad) {
            this.drivePort = drivePort;
            this.turnPort = turnPort;
            this.driveInverted = driveInverted;
            this.turnInverted = turnInverted;
            this.encoderPort = encoderPort;
            this.encoderOffsetRad = encoderOffsetRad;
            this.driveRatio = ModuleConstants.kDriveGearRatios[DriveConstants.kGear];
        }

        /** Motor rotations -> wheel meters. */
        public double driveRotToMeters() {
            return driveRatio * Math.PI * ModuleConstants.kWheelDiameterMeters;
        }

        /** Motor rotations -> wheel radians. */
        public double turnRotToRad() {
            return ModuleConstants.kTurnGearRatio * 2 * Math.PI;
        }
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.75);

        // Index 0 is unused filler so kGear reads 1-3 like the SDS chart.
        // Doubles matter: 225 / 32 as ints would silently become 7.
        public static final double[] kDriveGearRatios = {1, 225.0 / 32, 675.0 / 112, 675.0 / 128};
        public static final double kTurnGearRatio = 287.0 / 11;

        public static final Pid kTurnPid = new Pid(1, 0, 0);
        public static final Pid kDrivePid = new Pid(1, 0, 0);
    }

    public static final class DriveConstants {
        // Full stick speed, then slow mode while the right bumper is held.
        public static final double kSpeed = 5;
        public static final double kSlowSpeed = 1.5;

        // Radians per second.
        public static final double kTurnSpeed = 3 * Math.PI;
        public static final double kTurnSlowSpeed = Math.PI;

        // Point mode P gain: radians per second of correction per radian of heading error.
        public static final double kHeadingKp = 3;

        // Units: m.
        public static final double kTrackWidth = Units.inchesToMeters(21.75);
        public static final double kWheelBase = Units.inchesToMeters(21.75);

        public static final int kGear = 1;

        /** Module order everywhere in this code is FL, FR, BL, BR. */
        public static final SwerveDriveKinematics kKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL (+x, +y)
                        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // FR (+x, -y)
                        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // BL (-x, +y)
                        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // BR (-x, -y)

        public static final SwerveModulePreset kModuleFL =
                new SwerveModulePreset(
                        CAN.kDriveMotorPortFL, CAN.kTurnMotorPortFL, false, false, 0, 0);
        public static final SwerveModulePreset kModuleFR =
                new SwerveModulePreset(
                        CAN.kDriveMotorPortFR, CAN.kTurnMotorPortFR, false, false, 1, 0);
        public static final SwerveModulePreset kModuleBL =
                new SwerveModulePreset(
                        CAN.kDriveMotorPortBL, CAN.kTurnMotorPortBL, false, false, 2, 0);
        public static final SwerveModulePreset kModuleBR =
                new SwerveModulePreset(
                        CAN.kDriveMotorPortBR, CAN.kTurnMotorPortBR, false, false, 3, 0);

        /** Toe-in X so a pushed robot resists sliding; fronts and backs point opposite ways. */
        public static final SwerveModuleState[] kLockStates = {
            new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
            new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
            new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
            new SwerveModuleState(0, new Rotation2d(Math.PI / 4))
        };
    }
}
