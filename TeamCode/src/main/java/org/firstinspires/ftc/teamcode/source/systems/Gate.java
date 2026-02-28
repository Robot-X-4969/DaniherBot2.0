package org.firstinspires.ftc.teamcode.source.systems;

import org.firstinspires.ftc.teamcode.libs.components.XDriverStation;
import org.firstinspires.ftc.teamcode.libs.components.XServo;
import org.firstinspires.ftc.teamcode.libs.templates.XModule;
import org.firstinspires.ftc.teamcode.libs.templates.XOpMode;
import org.firstinspires.ftc.teamcode.libs.util.Scheduler;

public class Gate extends XModule {

    private XServo servo1;
    private XServo servo2;

    private boolean gateOpen = true;

    public Gate(XOpMode op) {

        super(op);

    }

    @Override
    public void init(Scheduler scheduler, XDriverStation driverStation) {

        super.init(scheduler, driverStation);

        servo1 = new XServo(op, "gateServo1", 0.0);
        servo1.init();
        servo1.setReversed(false);
        servo1.setPosition(0.0);

        servo2 = new XServo(op, "gateServo2", 0.0);
        servo2.init();
        servo2.setReversed(true);
        servo2.setPosition(0.0);


    }

    @Override
    public void loop(double deltaTime) {

        super.loop(deltaTime);

    }

    @Override
    public void control_loop() {

       if(op.getXDriverStation().getGamepad1().getX().wasPressed()){

          toggleGate();

       }

    }

    @Override
    public void stop(){

        servo1.setPosition(0.0);
        servo1.setPosition(0.0);

    }

    public void toggleGate(){

        gateOpen = !gateOpen;

        if(gateOpen){

            servo1.setPosition(0.0);
            servo2.setPosition(0.0);

        } else {

            servo1.setPosition(100 * (1.0 / 300.0));
            servo2.setPosition(100 * (1.0 / 300.0));

        }

    }

}
