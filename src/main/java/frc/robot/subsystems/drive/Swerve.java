package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

/** Four swerve modules, ordered FL, FR, BL, BR to match the kinematics. */
public class Swerve extends SubsystemBase {
    private final SwerveModule[] modules;
    // Keeps each module's reported distance continuous when it drops off and comes back.
    private final double[] positionOffsets;
    private final double[] lastPositionsMeters;
    private final boolean[] wasFailed;

    public Swerve(
            SwerveModuleIO frontLeft,
            SwerveModuleIO frontRight,
            SwerveModuleIO backLeft,
            SwerveModuleIO backRight) {
        modules =
                new SwerveModule[] {
                    new SwerveModule(frontLeft),
                    new SwerveModule(frontRight),
                    new SwerveModule(backLeft),
                    new SwerveModule(backRight)
                };
        positionOffsets = new double[modules.length];
        lastPositionsMeters = new double[modules.length];
        wasFailed = new boolean[modules.length];
    }

    public void setStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeed);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(states[i]);
        }
    }

    public void setStates(ChassisSpeeds speeds) {
        setStates(DriveConstants.kKinematics.toSwerveModuleStates(speeds));
    }

    public void lockWheels() {
        setStates(DriveConstants.kLockStates);
    }

    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Positions for the pose estimator. A failed module is replaced by the healthy average, so its
     * encoder leaves the calculation but the other three keep the pose integrating.
     */
    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        double distance = 0, cos = 0, sin = 0;
        int healthy = 0;

        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
            if (!modules[i].isFailed()) {
                distance += positions[i].distanceMeters;
                cos += positions[i].angle.getCos();
                sin += positions[i].angle.getSin();
                healthy++;
            }
        }

        // No healthy wheel left: every wheel freezes where it was instead of jumping to zero.
        double averageDistance = healthy == 0 ? 0 : distance / healthy;
        Rotation2d averageAngle = healthy == 0 ? new Rotation2d() : new Rotation2d(cos, sin);

        for (int i = 0; i < modules.length; i++) {
            boolean failed = modules[i].isFailed();
            double raw = failed ? averageDistance : positions[i].distanceMeters;
            // The estimator only sees deltas, so a jump here becomes a pose error it never undoes.
            // Re-anchor on each health change to keep the reported distance continuous.
            if (failed != wasFailed[i]) {
                positionOffsets[i] = lastPositionsMeters[i] - raw;
                wasFailed[i] = failed;
            }
            lastPositionsMeters[i] = raw + positionOffsets[i];
            positions[i] =
                    new SwerveModulePosition(
                            lastPositionsMeters[i], failed ? averageAngle : positions[i].angle);
        }
        return positions;
    }

    /** Detects module connection changes, then flips coast/brake so recovery needs no operator. */
    void updateModuleHealth() {
        for (SwerveModule module : modules) {
            boolean failed = !module.isConnected();
            if (failed != module.isFailed()) module.setFailed(failed);
        }
    }

    @Override
    public void periodic() {
        updateModuleHealth();

        SwerveModuleState[] states = getStates();
        boolean[] failed = new boolean[modules.length];
        for (int i = 0; i < modules.length; i++) {
            failed[i] = modules[i].isFailed();
        }

        Logger.recordOutput("Drive/ModuleStates", states);
        Logger.recordOutput(
                "Drive/ChassisSpeeds", DriveConstants.kKinematics.toChassisSpeeds(states));
        Logger.recordOutput("Drive/Failed", failed);
    }
}
