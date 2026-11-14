package frc.robot.subsystems.drive.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final SwerveModuleIO io;

    public SwerveModule(SwerveModuleIO io) {
        this.io = io;
    }

    public void setRotation(Rotation2d rotation) {
        io.setRotation(rotation);
    }

    public void setVelocity(double velocity) {
        io.setVelocity(velocity);
    }

    public void setState(SwerveModuleState state) {
        io.setVelocity(state.speedMetersPerSecond);
        io.setRotation(state.angle);
    }

    public double getVelocity() {
        return getState().speedMetersPerSecond;
    }

    public Rotation2d getRotation() {
        return getState().angle;
    }

    public SwerveModuleState getState() {
        return io.getState();
    }

    public SwerveModulePosition getPosition() {
        return io.getPosition();
    }

    public void stop() {
        io.stop();
    }
}
