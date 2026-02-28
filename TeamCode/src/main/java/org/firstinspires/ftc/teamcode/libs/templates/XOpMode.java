package org.firstinspires.ftc.teamcode.libs.templates;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.components.XDriverStation;
import org.firstinspires.ftc.teamcode.libs.util.Scheduler;

/**
 * Interface that allows modules to access core op mode functionality without having to check if the op is auto or teleop
 *
 * @author Gavin Farrell
 */
public interface XOpMode {
    HardwareMap getHardwareMap();
    Telemetry getTelemetry();
    Scheduler getScheduler();
    XDriverStation getXDriverStation();
    void registerModule(XModule module, XModuleManager.ModuleType type);

}
