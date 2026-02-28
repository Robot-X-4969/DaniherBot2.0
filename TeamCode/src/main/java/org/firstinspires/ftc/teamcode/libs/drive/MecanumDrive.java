package org.firstinspires.ftc.teamcode.libs.drive;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.components.XDriverStation;
import org.firstinspires.ftc.teamcode.libs.components.XIMU;
import org.firstinspires.ftc.teamcode.libs.components.XMotor;
import org.firstinspires.ftc.teamcode.libs.templates.XOpMode;
import org.firstinspires.ftc.teamcode.libs.templates.XModule;
import org.firstinspires.ftc.teamcode.libs.util.Scheduler;


public class MecanumDrive extends XModule {

    private XMotor frontLeft;
    private XMotor frontRight;
    private XMotor backRight;
    private XMotor backLeft;

    private XIMU orientationSensor;
    private Orientation lastOrientation;

    private double globalAngle;
    private double x;
    private double y;
    private double r;
    private double autoR;
    private double offset;
    private boolean orientationMode;
    private boolean allowedRotation;


    public MecanumDrive(XOpMode op) {

        super(op);

    }

    @Override
    public void init(Scheduler scheduler, XDriverStation driverStation) {

        super.init(scheduler, driverStation);

        frontLeft = new XMotor(op, "frontLeft");
        frontRight = new XMotor(op, "frontRight");
        backRight = new XMotor(op, "backRight");
        backLeft = new XMotor(op, "backLeft");

        frontLeft.init();
        frontRight.init();
        backRight.init();
        backLeft.init();

        frontRight.setReverse(true);
        backRight.setReverse(true);
        backLeft.setReverse(true);

        orientationSensor = new XIMU(op, "imu");
        orientationSensor.init();

        lastOrientation = new Orientation();

        orientationMode = true;

        offset = 0;
        globalAngle = 0;

        refreshOrientation();

        autoR = 0;

        allowedRotation = true;

    }

    @Override
    public void loop(double deltaTime) {

        super.loop(deltaTime);

        getHeadingAngle();

        final double robotAngle = orientationMode ? Math.toRadians(globalAngle - offset) : 0;

        final double joystickAngle = Math.atan2(-y, x);

        powerMotors(1.0, joystickAngle, robotAngle);

    }

    @Override
    public void displayTelemetry() {



    }

    @Override
    public void control_loop() {

        updateSticks();

        if(op.getXDriverStation().getGamepad1().getY().wasPressed()){

            orientationMode = !orientationMode;

        }

        if(op.getXDriverStation().getGamepad1().getX().wasPressed()){

            refreshOrientation();

        }

    }

    public void updateSticks() {

        x = op.getXDriverStation().getGamepad1().getLeftStickX();
        y = op.getXDriverStation().getGamepad1().getLeftStickY();
        r = allowedRotation ? op.getXDriverStation().getGamepad1().getRightStickX() : this.autoR;

    }

    public void refreshOrientation() {

        offset = globalAngle;

    }

    public void getHeadingAngle() {

        Orientation newOrientation = orientationSensor.getOrientation();

        double deltaAngle = newOrientation.firstAngle - lastOrientation.firstAngle;

        if (deltaAngle < -180) {

            deltaAngle += 360;

        } else if (deltaAngle > 180) {

            deltaAngle -= 360;

        }

        globalAngle -= deltaAngle;

        lastOrientation = newOrientation;

    }

    public void powerMotors(double power, double joystickAngle, double robotAngle) {

        double scaling = Math.pow(Math.max(Math.abs(x), Math.max(Math.abs(y), Math.abs(r))), 2) / ((x * x) + (y * y) + (r * r));

        final double magnitude = Math.sqrt((x * x) + (y * y));

        final double xPrime = magnitude * (Math.cos(robotAngle + joystickAngle));

        final double yPrime = -magnitude * (Math.sin(robotAngle + joystickAngle));

        frontLeft.setPower((yPrime - xPrime - r) * scaling * power);
        frontRight.setPower((yPrime + xPrime + r) * scaling * power);
        backLeft.setPower((yPrime + xPrime - r) * scaling * power);
        backRight.setPower((yPrime - xPrime + r) * scaling * power);

    }

    public void setAllowedRotation(boolean allowed) {

        this.allowedRotation = allowed;

    }

    public void setAutoR(double r) {

        this.autoR = r;

    }

}
