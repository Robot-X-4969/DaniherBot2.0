package org.firstinspires.ftc.teamcode.source.systems;

import org.firstinspires.ftc.teamcode.libs.components.XCamera;
import org.firstinspires.ftc.teamcode.libs.components.XDriverStation;
import org.firstinspires.ftc.teamcode.libs.components.XPinpoint;
import org.firstinspires.ftc.teamcode.libs.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.libs.templates.XOpMode;
import org.firstinspires.ftc.teamcode.libs.templates.XModule;
import org.firstinspires.ftc.teamcode.libs.util.Scheduler;

public class CameraSystem extends XModule {

    private XPinpoint pinpoint;
    private XCamera camera;

    private final MecanumDrive drive;

    private boolean isAligning;

    private double lastError;
    private double theta;
    private double targetHeading;
    private double wrappedError;
    private double error;
    public CameraSystem(XOpMode op, MecanumDrive drive, XPinpoint pinpoint) {

        super(op);

        this.drive = drive;

        this.pinpoint = pinpoint;

    }

    @Override
    public void init(Scheduler scheduler, XDriverStation driverStation){

        super.init(scheduler, driverStation);

        this.camera = new XCamera(op, "limelight");

        camera.init();

        this.camera.setPipeline(2);

        isAligning = false;

    }

    @Override
    public void start(){

        lastError = 0.0;

    }

    @Override
    public void loop(double deltaTime){

        super.loop(deltaTime);

        camera.loop();

        if(isAligning && camera.seesAprilTag(24)){

            autoAlign(camera.getTx(camera.getAprilTagIndex(24)), deltaTime);



        } else if (isAligning && !camera.seesAprilTag(24)) {

            isAligning = false;

            drive.setAllowedRotation(true);

        }


    }

    @Override
    public void control_loop()  {

        if(op.getXDriverStation().getGamepad1().getLeftTriggerPressure() >= 0.5 && !isAligning){

            isAligning = true;

            drive.setAllowedRotation(false);

        }

    }

    @Override
    public void displayTelemetry(){

        op.getTelemetry().addData("theta" , theta);
        op.getTelemetry().addData("target heading: ", targetHeading);
        op.getTelemetry().addData("wrapped error: ", wrappedError);
        op.getTelemetry().addData("error: ", error);

    }

    public void autoAlign(double xAngle, double deltaTime){

        double kP = 0.02;
        double kD = 0.0005;

        double error = xAngle;

        double P = error * kP;
        double D = ((error - lastError)) / deltaTime * kD;

        double power = P + D;

        if(Math.abs(xAngle) > 3.0){

            if(power > 0){

                power += 0.005;

            } else if (power < 0){

                power -= 0.005;

            }

            drive.setAutoR(power);

        } else {

            drive.setAutoR(0);

            drive.setAllowedRotation(true);

            isAligning = false;

        }

        lastError = error;

    }

    public XCamera getCamera(){

        return camera;

    }

    public void pinpointAutoAlign(int team){

        double x = pinpoint.getX();
        double y = pinpoint.getY();

        double heading = pinpoint.getHeading();

        double thetaToGoal;
        if(team == 0)
        {
            thetaToGoal = Math.toDegrees(Math.atan2(138.0 - x, -6.0 - y));
        } else {
            thetaToGoal = Math.toDegrees(Math.atan2(138.0 - x, -138 - y));
        }

        this.theta = thetaToGoal;

        double error = thetaToGoal - heading;
        this.error = error;

        /*
        while(error > 180.0){

            error -= 360.0;

        }

        while(error < -180.0){

            error += 360.0;

        }

         */

        this.wrappedError = error;


        double kP = 0.008;

        if(Math.abs(error) > 2.0){

            double power = error * kP;

            if(power > 0){

                drive.setAutoR(Math.min(-power, -0.1));

            } else if (power < 0) {

                drive.setAutoR(Math.max(-power, 0.1));

            }

        } else {

            drive.setAutoR(0);

            drive.setAllowedRotation(true);

            isAligning = false;

        }


    }

}
