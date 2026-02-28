package org.firstinspires.ftc.teamcode.source.opmodes.auton;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "BLUE_NEAR_ONESHOT", group = "auton")
public class AutonBlueNearOneShot extends XAuton {

    private enum StateMachine {
        DRIVE_START,
        SHOOT_1,
        LEAVE_LINE,
        STOP

    }

    private PoseStorage poseStorage;
    private StateMachine currentState = StateMachine.DRIVE_START;
    private PathChain driveStart;
    private PathChain leaveLine;

    private final Gate gate = new Gate(this);
    private Pose startPosition = new Pose(24, 120, Math.toRadians(135));
    private final XPinpoint pinpoint = new XPinpoint(this, 1.125, 4.625);
    private final MecanumDrive drive = new MecanumDrive(this);
    private final CameraSystem cameraSystem = new CameraSystem(this, drive, pinpoint);
    private final Flywheel flywheel = new Flywheel(this, null, pinpoint);
    private final IntakeSystem intakeSystem = new IntakeSystem(this);
    private final Spindexer spindexer = new Spindexer(this, gate);



    @Override
    public void init_modules(){

        pinpoint.init();

        registerModule(drive, XModuleManager.ModuleType.ACTIVE);
        registerModule(cameraSystem, XModuleManager.ModuleType.ACTIVE);
        registerModule(flywheel, XModuleManager.ModuleType.ACTIVE);
        registerModule(intakeSystem, XModuleManager.ModuleType.ACTIVE);
        registerModule(spindexer, XModuleManager.ModuleType.ACTIVE);
        registerModule(gate, XModuleManager.ModuleType.ACTIVE);

    }

    @Override
    public void initialize() {

        super.initialize();

        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPosition);

        buildPaths();

        gate.toggleGate();


    }

    public void buildPaths(){

        driveStart = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(24, 120, Math.toRadians(135.0)), new Pose(59, 85)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        leaveLine = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59, 85), new Pose(59, 59)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

    }

    @Override
    public void run(){

        super.run();

        telemetry.addData("x", pinpoint.getX());
        telemetry.addData("y", pinpoint.getY());
        telemetry.addData("heading", pinpoint.getHeading());

        telemetry.update();

        flywheel.loop(delta);

        follower.update();

        switch (currentState) {

            case DRIVE_START:


                follower.followPath(driveStart, true);
                currentState = StateMachine.SHOOT_1;


                break;

            case SHOOT_1:

                if (!follower.isBusy()) {

                    spindexer.burstFire();
                    currentState = StateMachine.LEAVE_LINE;

                }

                break;

            case LEAVE_LINE:

                if(!follower.isBusy() && !spindexer.getIsFiring()){

                    follower.followPath(leaveLine, true);
                    currentState = StateMachine.STOP;


                }

                break;

            default:

                PoseStorage.setCurrentPose(new Pose(pinpoint.getX(), pinpoint.getY(), pinpoint.getHeading() - Math.toRadians(90)));
                break;


        }

    }

}
