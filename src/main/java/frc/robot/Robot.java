// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private static final long kLogRetentionMillis = 7L * 24 * 60 * 60 * 1000;

    private final RobotContainer robotContainer;

    public Robot() {
        File logDir = new File(RobotBase.isReal() ? "/home/lvuser/logs" : "logs");
        deleteLogsOlderThan(logDir, System.currentTimeMillis() - kLogRetentionMillis);

        Logger.addDataReceiver(new WPILOGWriter(logDir.getPath()));
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();

        // Built after Logger.start() so construction-time logs are captured.
        robotContainer = new RobotContainer();
    }

    /** The Rio disk is small; a season of logs would fill it. */
    private static void deleteLogsOlderThan(File logDir, long cutoffMillis) {
        File[] oldLogs =
                logDir.listFiles(
                        file ->
                                file.getName().endsWith(".wpilog")
                                        && file.lastModified() < cutoffMillis);
        if (oldLogs == null) return;
        for (File log : oldLogs) log.delete();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
