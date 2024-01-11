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

        while (opModeIsActive()) {
            placeYellowServo.setPosition(0);
            sleep(10000);
            placeYellowServo.setPosition(1.0);
            sleep(200000);
        }
    }
}
