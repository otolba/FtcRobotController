package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "park blue", group = "linear autoMode")
public class ParkBlue extends RobotLinearOpMode{
    @Override
    public void runOpMode(){
        declareHardwareProperties();

        while (!isStarted() && !isStopRequested()){
            declareAutoVariables();
            telemetry.update();
        }

        while (opModeIsActive()){
            sleep(waitTime);

            if (close){
                if (parkMiddle){
                    encoderDrive(0.7, 40, MOVEMENT_DIRECTION.FORWARD);
                    encoderDrive(0.7, 30, MOVEMENT_DIRECTION.STRAFE_LEFT);
                }
                else{
                    encoderDrive(0.7, 2, MOVEMENT_DIRECTION.FORWARD);
                    encoderDrive(0.7, 30, MOVEMENT_DIRECTION.STRAFE_LEFT);
                }
            }
            else{
                if (parkMiddle){
                    if (far){
                        encoderDrive(0.7, 3, MOVEMENT_DIRECTION.FORWARD);
                        encoderDrive(0.7, 15, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                        encoderDrive(0.7, 40, MOVEMENT_DIRECTION.FORWARD);
                        encoderDrive(0.7, 50, MOVEMENT_DIRECTION.STRAFE_LEFT);
                    }
                    else{
                        encoderDrive(0.7, 3, MOVEMENT_DIRECTION.FORWARD);
                        encoderDrive(0.7, 50, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                        encoderDrive(0.7, 40, MOVEMENT_DIRECTION.FORWARD);
                        encoderDrive(0.7, 20, MOVEMENT_DIRECTION.STRAFE_LEFT);
                    }
                }
                else{
                    if (far){
                        encoderDrive(0.7, 3, MOVEMENT_DIRECTION.FORWARD);
                        encoderDrive(0.7, 15, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                        encoderDrive(0.7, 40, MOVEMENT_DIRECTION.FORWARD);
                        encoderDrive(0.7, 40, MOVEMENT_DIRECTION.STRAFE_LEFT);
                        encoderDrive(0.7, 40, MOVEMENT_DIRECTION.REVERSE);
                        encoderDrive(0.7, 15, MOVEMENT_DIRECTION.STRAFE_LEFT);
                    }
                    else{
                        encoderDrive(0.7, 3, MOVEMENT_DIRECTION.FORWARD);
                        encoderDrive(0.7, 15, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                        encoderDrive(0.7, 40, MOVEMENT_DIRECTION.FORWARD);
                        encoderDrive(0.7, 60, MOVEMENT_DIRECTION.STRAFE_LEFT);
                    }
                }
            }
        }
    }
}