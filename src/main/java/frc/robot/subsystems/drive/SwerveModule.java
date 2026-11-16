package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Vendor-neutral module logic: state optimization, health state and units already in SI. */
public class SwerveModule {
    private final SwerveModuleIO io;
    private boolean failed;

    SwerveModule(SwerveModuleIO io) {
        this.io = io;
    }

    /** Drives and steers toward a target, flipping it 180 degrees when that is the shorter path. */
    void setState(SwerveModuleState state) {
        if (failed) return; // a coasting wheel ignores commands until it reconnects
        // Copy first: optimize mutates, and callers may hand us a shared static array.
        SwerveModuleState target = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
        target.optimize(getState().angle);
        io.setDriveVelocity(target.speedMetersPerSecond);
        io.setTurnAngle(target.angle);
    }

    /** Failure means coast, no commands, and pose ignores this encoder. */
    void setFailed(boolean failed) {
        this.failed = failed;
        io.setCoast(failed);
    }

    boolean isFailed() {
        return failed;
    }

    boolean isConnected() {
        return io.isConnected();
    }

    SwerveModuleState getState() {
        return new SwerveModuleState(io.getDriveVelocity(), io.getTurnAngle());
    }

    SwerveModulePosition getPosition() {
        return new SwerveModulePosition(io.getDrivePositionMeters(), io.getTurnAngle());
    }

    void stop() {
        io.stop();
    }
}
