package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

class DriveMathTest {
    private static final double kDeadband = 0.1;

    @Test
    void curveKeepsSignAndKillsDeadband() {
        assertEquals(0, DriveMath.curve(0.05, kDeadband));
        assertEquals(0, DriveMath.curve(-0.05, kDeadband));
        assertEquals(1, DriveMath.curve(1, kDeadband));
        assertEquals(-1, DriveMath.curve(-1, kDeadband));
        // Cubic on the deadband-rescaled stick: half stick is under an eighth of full authority.
        assertEquals(
                Math.pow((0.5 - kDeadband) / (1 - kDeadband), 3),
                DriveMath.curve(0.5, kDeadband),
                1e-9);
    }

    @Test
    void stickHeadingMatchesFieldAxes() {
        assertEquals(0, DriveMath.stickHeading(1, 0, kDeadband).getRadians(), 1e-9);
        assertEquals(Math.PI / 2, DriveMath.stickHeading(0, 1, kDeadband).getRadians(), 1e-9);
        assertEquals(-Math.PI / 2, DriveMath.stickHeading(0, -1, kDeadband).getRadians(), 1e-9);
        assertEquals(
                Math.PI, Math.abs(DriveMath.stickHeading(-1, 0, kDeadband).getRadians()), 1e-9);
    }

    @Test
    void centeredStickHasNoHeading() {
        assertNull(DriveMath.stickHeading(0.05, -0.05, kDeadband));
    }
}
