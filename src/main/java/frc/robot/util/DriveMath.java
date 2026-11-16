package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/** Pure input math, kept static so it can be unit tested without a robot. */
public final class DriveMath {
    private DriveMath() {}

    /** Deadband, then cube: fine control near center, full authority at the stops, sign kept. */
    public static double curve(double stick, double deadband) {
        double value = MathUtil.applyDeadband(stick, deadband);
        return value * value * value;
    }

    /**
     * Field direction the right stick points at, or null while that stick sits inside the deadband.
     */
    public static Rotation2d stickHeading(double forward, double strafe, double deadband) {
        double x = MathUtil.applyDeadband(strafe, deadband);
        double y = MathUtil.applyDeadband(forward, deadband);
        if (x == 0 && y == 0) return null;
        return new Rotation2d(Math.atan2(x, y));
    }
}
