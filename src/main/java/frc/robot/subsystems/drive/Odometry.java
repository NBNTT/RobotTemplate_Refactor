package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

/** Pigeon heading plus wheel odometry, fused into one pose. Vision can be added later. */
public class Odometry extends SubsystemBase {
    private final Swerve swerve;
    private final Pigeon2 pigeon;
    private final SwerveDrivePoseEstimator estimator;
    private Rotation2d heading;
    private boolean gyroOk;

    public Odometry(Swerve swerve) {
        this.swerve = swerve;
        // Pigeon on the roboRIO CAN bus; add the bus name if it ever moves to a CANivore.
        pigeon = new Pigeon2(CAN.kPigeonPort);
        readGyro();
        estimator =
                new SwerveDrivePoseEstimator(
                        DriveConstants.kKinematics, heading, swerve.getPositions(), new Pose2d());
    }

    @Override
    public void periodic() {
        // A Pigeon read refreshes a status signal, so read once and let the rest use the cache.
        readGyro();
        estimator.update(heading, swerve.getPositions());
        Logger.recordOutput("Odometry/Pose", getPose());
        Logger.recordOutput("Odometry/GyroOk", gyroOk);
    }

    /** Orient: the way the robot points now becomes forward. The pose keeps its position. */
    public void reset() {
        pigeon.setYaw(0);
        readGyro();
        estimator.resetPosition(
                heading, swerve.getPositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    private void readGyro() {
        heading = pigeon.getRotation2d();
        gyroOk = pigeon.getYaw().getStatus().isOK();
    }

    public Rotation2d getHeading() {
        return heading;
    }

    /** Field-relative driving must not trust a heading that stopped updating. */
    public boolean isGyroOk() {
        return gyroOk;
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }
}
