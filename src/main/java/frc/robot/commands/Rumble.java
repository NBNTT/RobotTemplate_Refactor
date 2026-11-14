package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Rumble extends Command {
    private final CommandXboxController controller;
    private final RumbleType type;
    private final double strength;

    public Rumble(CommandXboxController controller, RumbleType type, double strength) {
        this.controller = controller;
        this.type = type;
        this.strength = strength;
    }

    @Override
    public void initialize() {
        controller.setRumble(type, strength);
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(type, 0);
    }
}
