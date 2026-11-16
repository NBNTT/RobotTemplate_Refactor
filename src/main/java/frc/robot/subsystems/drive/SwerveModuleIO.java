package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Hardware seam for one swerve module. Everything above this interface is vendor-neutral, so the
 * MK4n/SparkMax drive is replaced by writing one TalonFX class that implements these methods.
 */
public interface SwerveModuleIO {
    void setDriveVelocity(double metersPerSecond);

    void setTurnAngle(Rotation2d angle);

    /** Coast lets a failed wheel spin freely instead of dragging the robot. */
    void setCoast(boolean coast);

    void stop();

    double getDriveVelocity();

    double getDrivePositionMeters();

    Rotation2d getTurnAngle();

    /** False when the module's motor controllers stop answering on the CAN bus. */
    boolean isConnected();
}
