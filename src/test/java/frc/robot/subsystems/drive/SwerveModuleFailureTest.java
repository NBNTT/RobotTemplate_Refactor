package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.junit.jupiter.api.Test;

class SwerveModuleFailureTest {
    /** Minimal hardware stand-in so failure logic runs without motors. */
    private static class FakeModule implements SwerveModuleIO {
        boolean connected = true;
        boolean coast;
        double distance;
        double commandedVelocity;

        FakeModule(double distance) {
            this.distance = distance;
        }

        @Override
        public void setDriveVelocity(double metersPerSecond) {
            commandedVelocity = metersPerSecond;
        }

        @Override
        public void setTurnAngle(Rotation2d angle) {}

        @Override
        public void setCoast(boolean coast) {
            this.coast = coast;
        }

        @Override
        public void stop() {
            commandedVelocity = 0;
        }

        @Override
        public double getDriveVelocity() {
            return commandedVelocity;
        }

        @Override
        public double getDrivePositionMeters() {
            return distance;
        }

        @Override
        public Rotation2d getTurnAngle() {
            return new Rotation2d();
        }

        @Override
        public boolean isConnected() {
            return connected;
        }
    }

    @Test
    void failedModuleCoastsIsExcludedFromPoseThenRecovers() {
        FakeModule fl = new FakeModule(1);
        FakeModule fr = new FakeModule(2);
        FakeModule bl = new FakeModule(3);
        FakeModule br = new FakeModule(4);
        Swerve swerve = new Swerve(fl, fr, bl, br);

        fl.connected = false;
        swerve.updateModuleHealth();

        assertTrue(fl.coast, "failed module should coast");
        SwerveModulePosition[] positions = swerve.getPositions();
        assertEquals(3.0, positions[0].distanceMeters, 1e-9, "failed encoder is averaged out");
        assertEquals(2.0, positions[1].distanceMeters, 1e-9, "healthy modules are untouched");

        swerve.setStates(new ChassisSpeeds(1, 0, 0));
        assertEquals(0, fl.commandedVelocity, 1e-9, "failed module takes no commands");

        fl.connected = true;
        swerve.updateModuleHealth();

        assertFalse(fl.coast, "recovered module returns to brake");
        assertEquals(1.0, swerve.getPositions()[0].distanceMeters, 1e-9, "encoder is used again");
    }
}
