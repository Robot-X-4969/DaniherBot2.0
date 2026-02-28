package org.firstinspires.ftc.teamcode.libs.templates;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.components.XDriverStation;
import org.firstinspires.ftc.teamcode.libs.util.Scheduler;

/**
 * Base class for autonomous op modes. Provides core functionality such as module management and driver station access.
 *
 * @author Gavin Farrell
 */
public abstract class XAuton extends LinearOpMode implements XOpMode {

    private final XModuleManager manager = new XModuleManager(this);

    protected Scheduler scheduler;

    protected XDriverStation driverStation;

    protected Follower follower;

    protected double lastTime;

    protected double delta;

    @Override
    public void runOpMode(){

        initialize();

        waitForStart();

        lastTime = System.currentTimeMillis() / 1000.0;

        while (opModeIsActive()){

            scheduler.loop();

            run();

        }

    }
    public void init_modules(){


    }

    public void initialize(){

        driverStation = new XDriverStation(gamepad1, gamepad2);
        scheduler = new Scheduler();

        init_modules();

        for(XModule system : manager.getActiveModules()) {

            system.init(this.scheduler, this.driverStation);

        }

        for(XModule system : manager.getInactiveModules()){

            system.init(this.scheduler, this.driverStation);

        }


    }

    public void run(){

        double currentTime = System.currentTimeMillis() / 1000.0;
        delta = currentTime - lastTime;
        lastTime = currentTime;



    }

    @Override
    public void registerModule(XModule module, XModuleManager.ModuleType type){

        manager.register_module(module, type);

    }


    @Override
    public HardwareMap getHardwareMap() {

        return hardwareMap;

    }

    @Override
    public Telemetry getTelemetry() {

        return telemetry;

    }

    @Override
    public XDriverStation getXDriverStation(){

        return this.driverStation;

    }

    @Override
    public Scheduler getScheduler(){

        return this.scheduler;

    }

}
