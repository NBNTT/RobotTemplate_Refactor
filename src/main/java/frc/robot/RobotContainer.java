package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveTeleOp;
import frc.robot.subsystems.drive.Autos;
import frc.robot.subsystems.drive.Odometry;
import frc.robot.subsystems.drive.SparkMaxSwerveModule;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.DriveController;

/** Wires hardware to commands. Only the bindings listed here do anything. */
public class RobotContainer {
    private final Swerve swerve;
    private final Odometry odometry;
    private final Autos autos;
    private final DriveController controller = new DriveController(OIConstants.kDriverPort);

    public RobotContainer() {
        swerve =
                new Swerve(
                        new SparkMaxSwerveModule(DriveConstants.kModuleFL),
                        new SparkMaxSwerveModule(DriveConstants.kModuleFR),
                        new SparkMaxSwerveModule(DriveConstants.kModuleBL),
                        new SparkMaxSwerveModule(DriveConstants.kModuleBR));
        odometry = new Odometry(swerve);
        autos = new Autos();

        configureBindings();
    }

    private void configureBindings() {
        DriveTeleOp drive = new DriveTeleOp(swerve, odometry, controller);
        swerve.setDefaultCommand(drive);

        // D-pad picks the drive mode.
        controller.fieldMode().onTrue(runOnce(() -> drive.setMode(DriveTeleOp.Mode.FIELD)));
        controller.robotMode().onTrue(runOnce(() -> drive.setMode(DriveTeleOp.Mode.ROBOT)));
        controller.pointMode().onTrue(runOnce(() -> drive.setMode(DriveTeleOp.Mode.POINT)));

        controller.resetHeading().onTrue(runOnce(odometry::reset));
        controller.lockWheels().whileTrue(swerve.run(swerve::lockWheels));
    }
}
