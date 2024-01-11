package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "ServoTest", group = "linear autoMode")
public class ServoTest extends RobotLinearOpMode{

    Servo placeYellowServo;
    @Override
    public void runOpMode(){
        placeYellowServo = hardwareMap.get(Servo.class, "placeYellowServo");
        placeYellowServo.setDirection(Servo.Direction.FORWARD);
        waitForStart();


        telemetry.addData("Status", "running");
        telemetry.update();
        sleep(5000);
        placeYellowServo.setPosition(0);
        sleep(400);
        telemetry.addData("Status", "part 1 done");
        placeYellowServo.setPosition(1.0);
        sleep(400);
        stop();

    }
}
