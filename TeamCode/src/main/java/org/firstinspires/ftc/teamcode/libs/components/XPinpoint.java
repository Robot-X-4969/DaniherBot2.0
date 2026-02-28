package org.firstinspires.ftc.teamcode.libs.components;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.libs.templates.XOpMode;

public class XPinpoint {

    private final XOpMode op;
    private GoBildaPinpointDriver device;

    //offset of the odometry wheels from the center of the robot
    private final double xOffset;
    private final double yOffset;

    /**
     * Constructor for the XPinpoint class. Initializes the opmode and the offsets of the odometry wheels from the center of the robot.
     * @param op The opmode that this pinpoint is being used in
     * @param xOffset The offset of the odometry wheels from the center of the robot in the x direction (positive is forward, negative is backward)
     * @param yOffset The offset of the odometry wheels from the center of the robot in the y direction (positive is to the left, negative is to the right)
     */
    public XPinpoint(XOpMode op, double xOffset, double  yOffset){

        this.op = op;
        this.xOffset = xOffset;
        this.yOffset = yOffset;

    }

    /**
     * Initializes the pinpoint by retrieving it from the hardware map, setting the offsets, encoder resolution, encoder directions, and resetting the position and IMU.
     */
    public void init(){

        device = op.getHardwareMap().get(GoBildaPinpointDriver.class, "pinpoint");

        device.setOffsets(xOffset, yOffset, DistanceUnit.INCH);

        device.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        device.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        //device.resetPosAndIMU();

    }

    /**
     * Adds the current x and y coordinates and heading of the robot to telemetry.
     */
    public void getData(){

        op.getTelemetry().addData("X coord: ", getX());
        op.getTelemetry().addData("Y coord: ", getY());
        op.getTelemetry().addData("direction: ", getHeading());

    }

    /**
     * Updates the pinpoint's position and heading
     */
    public void update(){

        device.update();

    }

    //getters
    public Pose2D getPose() {

        return device.getPosition();

    }

    public double getX() {

        return getPose().getX(DistanceUnit.INCH);

    }

    public double getY() {

        return getPose().getY(DistanceUnit.INCH);

    }

    public double getHeading() {

        return getPose().getHeading(AngleUnit.DEGREES);

    }

    /**
     * Resets the pinpoint's position and heading to (0, 0, 0).
     */
    public void reset() {

        device.resetPosAndIMU();

    }

    public void setStartPose(double x, double y, double heading){

        device.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, heading));

    }

}
