package org.firstinspires.ftc.teamcode.libs.templates;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.components.XDriverStation;
import org.firstinspires.ftc.teamcode.libs.util.Scheduler;

public abstract class XTeleOp extends OpMode implements XOpMode {

    private final XModuleManager manager = new XModuleManager(this);

    private XDriverStation driverStation;

    private Scheduler scheduler;

    private double lastTime;


    public void init_modules(){


    }

    @Override
    public void init() {

        driverStation = new XDriverStation(gamepad1, gamepad2);
        scheduler = new Scheduler();

        init_modules();

        for(XModule module : manager.getActiveModules()){

            module.init(scheduler, driverStation);

        }

        for(XModule module : manager.getInactiveModules()){

            module.init(scheduler, driverStation);

        }

    }

    @Override
    public void init_loop() {

        scheduler.loop();

        driverStation.update();

        telemetry.update();

        if(driverStation.getGamepad1() != null) {

            telemetry.addData("GP1:", "Connected");

        } else {

            telemetry.addData("GP1:", "Disconnected");

        }

        for (XModule module : manager.getActiveModules()) {

            module.init_loop();

        }

    }

    @Override
    public void start() {

        for(XModule module: manager.getActiveModules()){

            module.start();

        }

        lastTime = System.currentTimeMillis() / 1000.0;

    }

    @Override
    public void loop() {

        scheduler.loop();

        double currentTime = System.currentTimeMillis() / 1000.0;
        double deltaTime = currentTime - lastTime;

        if (driverStation != null) driverStation.update();

        for(XModule module : manager.getActiveModules()){

            module.loop(deltaTime);

        }

        displayTelemetry();

        lastTime = currentTime;

    }

    @Override
    public void stop() {

        for(XModule module : manager.getActiveModules()){

            module.stop();

        }

    }

    public void displayTelemetry(){

        telemetry.update();

    }

    @Override
    public void registerModule(XModule module, XModuleManager.ModuleType type){

        manager.register_module(module, type);

    }

    @Override
    public Scheduler getScheduler() {

        return scheduler;

    }

    @Override
    public XDriverStation getXDriverStation() {

        return driverStation;

    }

    @Override
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    @Override
    public Telemetry getTelemetry() {
        return telemetry;
    }

}
