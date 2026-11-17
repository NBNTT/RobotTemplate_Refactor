// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.util.Arrays;
import java.util.Comparator;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private static final int kLogsToKeep = 20;

    private final RobotContainer robotContainer;

    public Robot() {
        File logDir = new File(RobotBase.isReal() ? "/home/lvuser/logs" : "logs");
        trimOldLogs(logDir);

        Logger.addDataReceiver(new WPILOGWriter(logDir.getPath()));
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();

        // Built after Logger.start() so construction-time logs are captured.
        robotContainer = new RobotContainer();
    }

    /** The Rio disk is small; keep the newest runs and delete the rest. */
    private static void trimOldLogs(File logDir) {
        File[] logs = logDir.listFiles(file -> file.getName().endsWith(".wpilog"));
        if (logs == null || logs.length <= kLogsToKeep) return;
        Arrays.sort(logs, Comparator.comparingLong(File::lastModified));
        for (int i = 0; i < logs.length - kLogsToKeep; i++) logs[i].delete();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
