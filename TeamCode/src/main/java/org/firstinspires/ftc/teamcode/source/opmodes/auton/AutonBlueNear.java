package org.firstinspires.ftc.teamcode.source.opmodes.auton;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.components.XPinpoint;
import org.firstinspires.ftc.teamcode.libs.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.libs.templates.XAuton;
import org.firstinspires.ftc.teamcode.libs.templates.XModuleManager;
import org.firstinspires.ftc.teamcode.libs.tuning.PedroConstants;
import org.firstinspires.ftc.teamcode.libs.util.PoseStorage;
import org.firstinspires.ftc.teamcode.source.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.source.systems.Flywheel;
import org.firstinspires.ftc.teamcode.source.systems.Gate;
import org.firstinspires.ftc.teamcode.source.systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.source.systems.Spindexer;

@Autonomous(name = "BLUE_NEAR", group = "auton")
public class AutonBlueNear extends XAuton {
    private enum StateMachine {
        DRIVE_START,
        SHOOT_1,
        COLLECT_BALLS_1,
        RESET_1,
        SHOOT_2,
        COLLECT_BALLS_2,
        RESET_2,
        SHOOT_3,
        STOP
    }
    private TelemetryManager panelsTelemetry;
    private StateMachine currentState = StateMachine.DRIVE_START;
    private PathChain driveStart;
    private PathChain collectBalls1;
    private PathChain reset1;
    private PathChain collectBalls2;
    private PathChain reset2;

    private double lastTime = 0.0;

    private final Pose startPosition = new Pose(24, 120, Math.toRadians(135.0));

    Gate gate = new Gate(this);
    XPinpoint pinpoint = new XPinpoint(this, 1.125, 4.625);
    MecanumDrive drive = new MecanumDrive(this);
    CameraSystem cameraSystem = new CameraSystem(this, drive, pinpoint);
    Flywheel flywheel = new Flywheel(this, cameraSystem, pinpoint);
    Spindexer spindexer = new Spindexer(this, gate);
    IntakeSystem intakeSystem = new IntakeSystem(this);


    public void buildPaths(){

        driveStart = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(24, 120), new Pose(59, 80)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        collectBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59, 80), new Pose(42, 80)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(new Pose(42, 80), new Pose(37, 80)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(new Pose(42, 80), new Pose(32, 80)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(new Pose(42, 80), new Pose(27, 80)))
                .setTangentHeadingInterpolation()
                .build();

        reset1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(19.000, 83.808), new Pose(59.456, 83.808)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                .build();

        collectBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.456, 83.808), new Pose(41.392, 59.672)))
                .setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))
                .addPath(new BezierLine(new Pose(41.392, 59.672), new Pose(19.000, 59.672)))
                .setTangentHeadingInterpolation()
                .build();

        reset2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(19.000, 59.672), new Pose(59.456, 83.808)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                .build();

    }

    @Override
    public void init_modules(){

        pinpoint.init();
        registerModule(drive, XModuleManager.ModuleType.ACTIVE);
        registerModule(cameraSystem, XModuleManager.ModuleType.ACTIVE);
        registerModule(flywheel, XModuleManager.ModuleType.ACTIVE);
        registerModule(spindexer, XModuleManager.ModuleType.ACTIVE);
        registerModule(intakeSystem, XModuleManager.ModuleType.ACTIVE);
        registerModule(gate, XModuleManager.ModuleType.ACTIVE);

    }

    @Override
    public void initialize(){

        super.initialize();

        //pinpoint.setStartPose(20.480, 122.432, Math.toRadians(144.0));

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPosition);

        buildPaths();

    }

    @Override
    public void run(){

        super.run();

        intakeSystem.loop(delta);

        flywheel.loop(delta);

        follower.update();

        switch(currentState){

            case DRIVE_START:

                follower.followPath(driveStart, true);

                currentState = StateMachine.SHOOT_1;

                break;

            case SHOOT_1:

                if(!follower.isBusy()){

                    telemetry.addLine("shot once");

                    spindexer.burstFire();

                    currentState = StateMachine.COLLECT_BALLS_1;

                }

                break;

            case COLLECT_BALLS_1:

                if(!follower.isBusy() && !spindexer.getIsFiring()){

                    scheduler.scheduleEvent(400, () -> follower.setMaxPower(0.5));
                    scheduler.scheduleEvent(800, () -> follower.setMaxPower(0.3));
                    scheduler.scheduleEvent(1800, () -> spindexer.incrementSpindexer(true));
                    scheduler.scheduleEvent(2000, () -> spindexer.incrementSpindexer(true));
                    scheduler.scheduleEvent(2900, () -> gate.toggleGate());

                    //scheduler.scheduleEvent(1000, () -> follower.setMaxPower(0));
                    //scheduler.scheduleEvent(1100, () -> follower.setMaxPower(0.5));
                    //scheduler.scheduleEvent(1900, () -> spindexer.incrementSpindexer(true));
                    //scheduler.scheduleEvent(1200, () -> spindexer.incrementSpindexer(true));


                    telemetry.addLine("collecting");
                    follower.followPath(collectBalls1, true);
                    currentState = StateMachine.RESET_1;

                }

                break;

            case RESET_1:

                if(!follower.isBusy()){

                    telemetry.addLine("resetting");
                    follower.followPath(reset1, true);
                    currentState = StateMachine.SHOOT_2;

                }

                break;

            case SHOOT_2:

                if(!follower.isBusy()){

                    spindexer.burstFire();
                    telemetry.addLine("shot twice");
                    currentState = StateMachine.COLLECT_BALLS_2;

                }

                break;

            case COLLECT_BALLS_2:

                if(!follower.isBusy() && !spindexer.getIsFiring()){

                    telemetry.addLine("collecting");
                    follower.followPath(collectBalls2, true);
                    currentState = StateMachine.RESET_2;

                }

                break;

            case RESET_2:

                if(!follower.isBusy()){

                    telemetry.addLine("resetting");
                    follower.followPath(reset2, true);
                    currentState = StateMachine.SHOOT_3;

                }

                break;

            case SHOOT_3:

                if(!follower.isBusy()){

                    telemetry.addLine("shot thrice");
                    spindexer.burstFire();

                    //PoseStorage.setCurrentPose(pinpoint.getPose());

                    currentState = StateMachine.STOP;

                }

                break;


            case STOP:

                if(!follower.isBusy() && !spindexer.getIsFiring()){

                    telemetry.addLine("stopped");

                }

                break;

        }

    }

}
