package org.firstinspires.ftc.teamcode.libs.util;

import com.pedropathing.geometry.Pose;

public abstract class PoseStorage {

    static Pose currentPose = new Pose(0,0,0);

    public static Pose getCurrentPose() {

        return currentPose;

    }

    public static void setCurrentPose(Pose newPose) {

        currentPose = newPose;

    }

}
