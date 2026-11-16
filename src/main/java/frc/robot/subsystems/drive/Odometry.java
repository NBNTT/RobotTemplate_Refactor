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

    public Odometry(Swerve swerve) {
        this.swerve = swerve;
        pigeon = new Pigeon2(CAN.kPigeonPort);
        estimator =
                new SwerveDrivePoseEstimator(
                        DriveConstants.kKinematics,
                        getHeading(),
                        swerve.getPositions(),
                        new Pose2d());
    }

    @Override
    public void periodic() {
        // swerve.getPositions() already swaps out failed module encoders.
        estimator.update(getHeading(), swerve.getPositions());
        Logger.recordOutput("Odometry/Pose", getPose());
    }

    /** Back button: field forward is now robot forward and the pose starts again at the origin. */
    public void reset() {
        pigeon.setYaw(0);
        estimator.resetPosition(getHeading(), swerve.getPositions(), new Pose2d());
    }

    public Rotation2d getHeading() {
        return pigeon.getRotation2d();
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }
}
