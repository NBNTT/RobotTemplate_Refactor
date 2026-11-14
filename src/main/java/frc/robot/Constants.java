package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class FieldConstants {
        public static final boolean fieldOriented = true;
    }

    public static class OIConstants {
        public static final ControllerType controllerTypeDriver = ControllerType.XBOX;
        public static final ControllerType controllerTypeOperator = ControllerType.XBOX;

        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double kDeadband = 0.11;
        public static final double kRumbleStrength = 0.025;
    }

    public static class SwerveModulePreset {
        public final int drivePort, turnPort, encoderPort;
        public final double driveRatio, encoderOffset;
        public final boolean driveInverted, turnInverted;

        public SwerveModulePreset(
                int drivePort,
                int turnPort,
                boolean driveInverted,
                boolean turnInverted,
                int encoderPort,
                double encoderOffset) {
            this.drivePort = drivePort;
            this.turnPort = turnPort;

            this.encoderPort = encoderPort;
            this.encoderOffset = encoderOffset;

            this.driveRatio = SwerveModuleConstants.kDriveGearRatios[DriveConstants.kGear];
            this.driveInverted = driveInverted;
            this.turnInverted = turnInverted;
        }

        // units: rot -> m
        public double driveRot2Meters() {
            return driveRatio * Math.PI * SwerveModuleConstants.kWheelDiameter;
        }

        // units: rot -> rad
        public double turnRot2Rads() {
            return SwerveModuleConstants.kTurnGearRatio * Math.PI * 2;
        }

        // units: rot/min -> m/s
        public double driveRPM2MetersPerSec() {
            return driveRot2Meters() / 60;
        }

        // units: rot/min -> rad/s
        public double turnRPM2RadsPerSec() {
            return turnRot2Rads() * Math.PI * 2;
        }
    }

    public static class SwerveModulePIDValues {
        public final double turnP, turnI, turnD, driveP, driveI, driveD;

        public SwerveModulePIDValues(
                double turnP,
                double turnI,
                double turnD,
                double driveP,
                double driveI,
                double driveD) {
            this.turnP = turnP;
            this.turnI = turnI;
            this.turnD = turnD;

            this.driveP = driveP;
            this.driveI = driveI;
            this.driveD = driveD;
        }
    }

    public static final class SwerveModuleConstants {
        // units: m
        public static final double kWheelDiameter = Units.inchesToMeters(3.75);
        // units: m/s
        public static final double kMaxSpeed = 12;

        public static final double[] kDriveGearRatios = {
            1, // Filler
            225 / 32, 675 / 112, 675 / 128
        };
        public static final double kTurnGearRatio = 287 / 11;

        public static final SwerveModulePIDValues kSparkMaxPID =
                new SwerveModulePIDValues(1, 0, 0, 1, 0, 0);
        public static final SwerveModulePIDValues kTalonFxPID =
                new SwerveModulePIDValues(1, 0, 0, 1, 0, 0);
    }

    public static class DriveConstants {
        // units: m/s
        public static final double kDriveMaxSpeed = 5;
        public static final double kDriveSpeed = 5;
        public static final double kDriveSlowSpeed = 1.5;

        // units: rad/s
        public static final double kTurnMaxSpeed = 5 * Math.PI;
        public static final double kTurnSpeed = 3 * Math.PI;
        public static final double kTurnSlowSpeed = 1 * Math.PI;

        // units: m
        public static final double kTrackWidth = Units.inchesToMeters(21.75);
        public static final double kWheelBase = Units.inchesToMeters(21.75);

        public static final int kGear = 1;

        public static final SwerveDriveKinematics kDriveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front left (+/+)
                        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front right (+/-)
                        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back left (-/+)
                        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Back right (-/-)

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

        /*
        // units: m/s
        public static final double kMaxSpeed = 5;
        // units: rad/s
        public static final double kMaxAngularSpeed = 3 * Math.PI;

        // units: m/s/s
        public static final double kMaxAcceleration = 5;
        // units: rad/s/s
        public static final double kMaxAngularAcceleration = 3 * Math.PI;
         */

        // units: kg
        public static final double kRobotMass = 50;
        // units: kg/m/m
        public static final double kRobotMOI = 7.0;

        public static final SwerveModuleState[] kLockStates =
                new SwerveModuleState[] {
                    new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                    new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
                    new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                    new SwerveModuleState(0, new Rotation2d(-Math.PI / 4))
                };
    }

    public static class CAN {
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

    public static enum ControllerType {
        XBOX,
        LOGITECH,
        JOYSTICK
    }
}
