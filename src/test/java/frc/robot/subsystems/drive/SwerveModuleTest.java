package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import org.junit.jupiter.api.Test;

class SwerveModuleTest {
    /** Minimal hardware stand-in so module logic runs without motors. */
    private static class FakeModule implements SwerveModuleIO {
        boolean connected = true;
        boolean coast;
        double distance;
        double commandedVelocity;
        Rotation2d commandedAngle = new Rotation2d();

        FakeModule(double distance) {
            this.distance = distance;
        }

        @Override
        public void setDriveVelocity(double metersPerSecond) {
            commandedVelocity = metersPerSecond;
        }

        @Override
        public void setTurnAngle(Rotation2d angle) {
            commandedAngle = angle;
        }

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
    void oneWheelTurnIsOneCircumference() {
        // Gear times meters per motor turn must equal the wheel circumference.
        double gear = ModuleConstants.kDriveGearRatios[DriveConstants.kGear];
        assertEquals(
                Math.PI * ModuleConstants.kWheelDiameterMeters,
                DriveConstants.kModuleFL.driveRotToMeters() * gear,
                1e-9);
    }

    @Test
    void targetOverNinetyDegreesFlipsTheShortWay() {
        FakeModule io = new FakeModule(0);
        SwerveModule module = new SwerveModule(io);

        module.setState(new SwerveModuleState(1, Rotation2d.fromDegrees(170)));

        assertEquals(-10, io.commandedAngle.getDegrees(), 1e-9, "steers 10 degrees, not 170");
        assertEquals(-1, io.commandedVelocity, 1e-9, "drive direction flips with the wheel");
    }

    @Test
    void failedModuleCoastsAndTakesNoCommands() {
        FakeModule fl = new FakeModule(1);
        FakeModule fr = new FakeModule(2);
        FakeModule bl = new FakeModule(3);
        FakeModule br = new FakeModule(4);
        Swerve swerve = new Swerve(fl, fr, bl, br);
        swerve.getPositions(); // seed last positions

        fl.connected = false;
        swerve.updateModuleHealth();

        assertTrue(fl.coast, "failed module should coast");
        assertEquals(1.0, swerve.getPositions()[0].distanceMeters, 1e-9, "no jump when it drops");

        swerve.setStates(new ChassisSpeeds(1, 0, 0));
        assertEquals(0, fl.commandedVelocity, 1e-9, "failed module takes no commands");

        fl.connected = true;
        swerve.updateModuleHealth();

        assertFalse(fl.coast, "recovered module returns to brake");
    }

    @Test
    void poseStaysContinuousAcrossFailureAndRecovery() {
        FakeModule fl = new FakeModule(1);
        FakeModule fr = new FakeModule(1);
        FakeModule bl = new FakeModule(1);
        FakeModule br = new FakeModule(1);
        Swerve swerve = new Swerve(fl, fr, bl, br);
        swerve.getPositions();

        fl.connected = false;
        swerve.updateModuleHealth();
        assertEquals(1.0, swerve.getPositions()[0].distanceMeters, 1e-9, "no jump on failure");

        // Healthy wheels roll 0.5 m while FL is dead; its reported distance follows the average.
        fr.distance += 0.5;
        bl.distance += 0.5;
        br.distance += 0.5;
        fl.distance = 0; // a dead encoder reads zero
        assertEquals(
                1.5, swerve.getPositions()[0].distanceMeters, 1e-9, "average motion while failed");

        fl.connected = true;
        fl.distance = 0.2;
        swerve.updateModuleHealth();
        assertEquals(1.5, swerve.getPositions()[0].distanceMeters, 1e-9, "no jump on recovery");

        fl.distance = 0.3;
        assertEquals(1.6, swerve.getPositions()[0].distanceMeters, 1e-9, "real encoder used again");
    }
}
