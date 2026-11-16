package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * 8BitDo Ultimate in XInput mode reads as an Xbox pad. Buttons are named by job, not by letter, so
 * swapping to a different controller means editing only this class.
 */
public class DriveController {
    private final CommandXboxController hid;

    public DriveController(int port) {
        hid = new CommandXboxController(port);
    }

    /** Positive is forward/left, matching WPILib's +x/+y field axes. */
    public double forward() {
        return -hid.getLeftY();
    }

    public double strafe() {
        return -hid.getLeftX();
    }

    public double turn() {
        return -hid.getRightX();
    }

    /** Right stick as a vector; only point mode reads it, where its angle is the target heading. */
    public double rightForward() {
        return -hid.getRightY();
    }

    public double rightStrafe() {
        return -hid.getRightX();
    }

    public boolean slowMode() {
        return hid.getHID().getRightBumperButton();
    }

    public Trigger fieldMode() {
        return hid.povUp();
    }

    public Trigger robotMode() {
        return hid.povDown();
    }

    public Trigger pointMode() {
        return hid.povRight();
    }

    public Trigger resetHeading() {
        return hid.back();
    }

    public Trigger lockWheels() {
        return hid.a();
    }
}
