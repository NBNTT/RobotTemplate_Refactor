package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.Odometry;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.DriveController;
import frc.robot.util.DriveMath;

/** Default teleop drive: cubic left stick for translation, right X for rotation or point mode. */
public class DriveTeleOp extends Command {
    /**
     * FIELD follows the field, ROBOT follows the chassis, POINT faces the direction of the right
     * stick.
     */
    public enum Mode {
        FIELD,
        ROBOT,
        POINT
    }

    private final Swerve swerve;
    private final Odometry odometry;
    private final DriveController controller;
    private Mode mode = Mode.FIELD;
    private Rotation2d pointHeading = new Rotation2d();
    private boolean gyroWasOk = true;

    public DriveTeleOp(Swerve swerve, Odometry odometry, DriveController controller) {
        this.swerve = swerve;
        this.odometry = odometry;
        this.controller = controller;
        addRequirements(swerve);
    }

    /** D-pad selects the mode; changing mode never interrupts driving. */
    public void setMode(Mode mode) {
        this.mode = mode;
        if (mode == Mode.POINT) pointHeading = odometry.getHeading(); // no snap on entry
    }

    /** Back button: drop the held point target and re-zero the field. */
    public void resetHeading() {
        pointHeading = new Rotation2d();
        odometry.reset();
    }

    @Override
    public void execute() {
        // The default command keeps running in auto; sticks must not drive the robot then.
        if (!DriverStation.isTeleopEnabled()) return;

        boolean slow = controller.slowMode();
        double speed = slow ? DriveConstants.kSlowSpeed : DriveConstants.kMaxSpeed;
        double turnSpeed = slow ? DriveConstants.kTurnSlowSpeed : DriveConstants.kTurnSpeed;

        double forward = DriveMath.curve(controller.forward(), OIConstants.kDeadband) * speed;
        double strafe = DriveMath.curve(controller.strafe(), OIConstants.kDeadband) * speed;
        double turn = DriveMath.curve(controller.turn(), OIConstants.kDeadband) * turnSpeed;

        // Red drivers face the other way, so their "forward" is the field's backward.
        double fieldSign =
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -1 : 1;

        boolean gyroOk = odometry.isGyroOk();
        if (!gyroOk || !gyroWasOk)
            pointHeading = odometry.getHeading(); // never resume an old target
        gyroWasOk = gyroOk;

        // A dead gyro means no field frame, so every mode falls back to robot-relative.
        ChassisSpeeds speeds =
                switch (gyroOk ? mode : Mode.ROBOT) {
                    case ROBOT -> new ChassisSpeeds(forward, strafe, turn);
                    case FIELD -> ChassisSpeeds.fromFieldRelativeSpeeds(
                            forward * fieldSign, strafe * fieldSign, turn, odometry.getHeading());
                    case POINT -> pointAt(
                            forward * fieldSign, strafe * fieldSign, turnSpeed, fieldSign < 0);
                };
        swerve.setStates(speeds);
    }

    /**
     * Right stick angle is an absolute field heading, so the robot turns to face it instead of
     * spinning at a stick-dependent rate. Centering the stick holds the last heading.
     */
    private ChassisSpeeds pointAt(
            double forward, double strafe, double turnSpeed, boolean redAlliance) {
        Rotation2d target =
                DriveMath.stickHeading(
                        controller.rightForward(), controller.rightStrafe(), OIConstants.kDeadband);
        if (target != null) {
            // Stick up means "face the other wall" on red, so the field target turns half a circle.
            pointHeading = redAlliance ? target.plus(Rotation2d.kPi) : target;
        }

        double error = pointHeading.minus(odometry.getHeading()).getRadians();
        double omega = MathUtil.clamp(DriveConstants.kHeadingKp * error, -turnSpeed, turnSpeed);
        return ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, omega, odometry.getHeading());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
