package org.firstinspires.ftc.teamcode.source.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.libs.templates.XTeleOp;

import org.firstinspires.ftc.teamcode.libs.templates.XModuleManager;
import org.firstinspires.ftc.teamcode.source.systems.CameraSystem;

@TeleOp (name = "Alignment Test", group = "Tests")
public class AlignmentTest extends XTeleOp {

    MecanumDrive drive = new MecanumDrive(this);
    //CameraSystem camera = new CameraSystem(this, drive, null);

    @Override
    public void init_modules() {

        registerModule(drive, XModuleManager.ModuleType.ACTIVE);
        //registerModule(camera, XModuleManager.ModuleType.ACTIVE);

    }

    @Override
    public void displayTelemetry() {

        super.displayTelemetry();

        drive.displayTelemetry();
        //camera.displayTelemetry();

    }


}
