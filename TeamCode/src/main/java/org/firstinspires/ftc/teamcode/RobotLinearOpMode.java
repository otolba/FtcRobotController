package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.text.method.MovementMethod;
import android.view.View;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.ArrayList;
import java.util.List;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


/**
 * This file is where each method should be written or referenced. Then when writing Autos and TeleOps,
 * programmers need to make files that extends RobotLinearOpMode instead of extending LinearOpMode.
 */

@Autonomous(name="RobotLinearOpMode", group="Linear Opmode")
@Disabled
public abstract class RobotLinearOpMode extends LinearOpMode {

    // Construction //
    public RobotLinearOpMode() {

    }

    int cameraMonitorViewId;
    OpenCvWebcam webcam;
    BluePropDetector.SkystoneDeterminationPipeline pipelineBlue;
    RedPropDetector.SkystoneDeterminationPipeline pipelineRed;
    BluePropDetector.SkystoneDeterminationPipeline.SkystonePosition snapshotAnalysis = BluePropDetector.SkystoneDeterminationPipeline.SkystonePosition.LEFT; // default
    RedPropDetector.SkystoneDeterminationPipeline.SkystonePosition snapshotAnalysisRed = RedPropDetector.SkystoneDeterminationPipeline.SkystonePosition.LEFT; // default
    //Declaration of drive motors
    DcMotor rightFrontDriveMotor;
    DcMotor leftFrontDriveMotor;
    DcMotor rightBackDriveMotor;
    DcMotor leftBackDriveMotor;
    DcMotor liftMotor;
    DcMotor intakeMotor;
    NormalizedColorSensor colorSensorLeft;
    NormalizedColorSensor colorSensorRight;
    Servo placePurplePixelLeft;
    Servo placePurplePixelRight;

    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    boolean USE_WEBCAM = false;  // true for webcam, false for phone camera
    boolean placingPixel = false;
    boolean searching;
    boolean aTagSeen = false;
    private ElapsedTime runtime = new ElapsedTime();

    boolean aWasPressed = false;
    boolean bWasPressed = false;
    boolean xWasPressed = false;
    boolean yWasPressed = false;
    boolean rBumperWasPressed = false;
    boolean lBumperWasPressed = false;
    boolean close = false;
    boolean far = false;
    boolean parkCorner = false;
    boolean parkMiddle = false;
    long waitTime = 0;

    public void encoderDrive(double power, double inches, MOVEMENT_DIRECTION movement_direction) {
        //Specifications of hardware
        final double WHEEL_DIAMETER_INCHES = 3.77953;
        final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * 3.141592653589793);
        final double GEAR_RATIO = 19.2;
        final double COUNTS_PER_ROTATION_AT_MOTOR = 28;
        final double TICKS_PER_ROTATION = (GEAR_RATIO * COUNTS_PER_ROTATION_AT_MOTOR);
        final double TICKS_PER_INCH = (TICKS_PER_ROTATION) / (WHEEL_CIRCUMFERENCE_INCHES);

        //Target # of ticks for each motor
        int leftFrontTarget;
        int rightFrontTarget;
        int leftBackTarget;
        int rightBackTarget;

        //Resets motor encoders to 0 ticks
        leftFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets the target # of ticks by intaking the number of desired inches of movement and converting to ticks
        leftFrontTarget = leftFrontDriveMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        rightFrontTarget = rightFrontDriveMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        leftBackTarget = leftBackDriveMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        rightBackTarget = rightBackDriveMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);

        if (movement_direction == MOVEMENT_DIRECTION.REVERSE) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(leftFrontTarget);
            rightFrontDriveMotor.setTargetPosition(rightFrontTarget);
            leftBackDriveMotor.setTargetPosition(leftBackTarget);
            rightBackDriveMotor.setTargetPosition(rightBackTarget);

            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);

            while (leftFrontDriveMotor.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            leftFrontDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);


        }

        if (movement_direction == MOVEMENT_DIRECTION.FORWARD) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(-leftFrontTarget);
            rightFrontDriveMotor.setTargetPosition(-rightFrontTarget);
            leftBackDriveMotor.setTargetPosition(-leftBackTarget);
            rightBackDriveMotor.setTargetPosition(-rightBackTarget);

            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);

            while (leftFrontDriveMotor.isBusy() && opModeIsActive()) {

            }
            //Kills the motors to prepare for next call of method
            leftFrontDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
        }

        if (movement_direction == MOVEMENT_DIRECTION.STRAFE_LEFT) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(leftFrontTarget * 2);
            rightFrontDriveMotor.setTargetPosition(-rightFrontTarget * 2);
            leftBackDriveMotor.setTargetPosition(-leftBackTarget * 2);
            rightBackDriveMotor.setTargetPosition(rightBackTarget * 2);


            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(-power);
            rightBackDriveMotor.setPower(power);

            while (leftFrontDriveMotor.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            leftFrontDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
        }

        if (movement_direction == MOVEMENT_DIRECTION.STRAFE_RIGHT) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(-leftFrontTarget * 2);
            rightFrontDriveMotor.setTargetPosition(rightFrontTarget * 2);
            leftBackDriveMotor.setTargetPosition(leftBackTarget * 2);
            rightBackDriveMotor.setTargetPosition(-rightBackTarget * 2);

            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);

            while (leftFrontDriveMotor.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            leftFrontDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
        }

        leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Kills the motors to prepare for next call of method
        leftFrontDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
        leftBackDriveMotor.setPower(0);
        rightBackDriveMotor.setPower(0);
    }

    public void encoderLift(double power, double inches, TeleOp2.LIFT_DIRECTION lift_direction) {

        //Specifications of hardware
        final double WHEEL_DIAMETER_INCHES = 3.77953;
        final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * 3.141592653589793);
        final double GEAR_RATIO = 19.2;
        final double COUNTS_PER_ROTATION_AT_MOTOR = 28;
        final double TICKS_PER_ROTATION = (GEAR_RATIO * COUNTS_PER_ROTATION_AT_MOTOR);
        final double TICKS_PER_INCH = (TICKS_PER_ROTATION) / (WHEEL_CIRCUMFERENCE_INCHES);

        int liftTarget;

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftTarget = liftMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);


        if (lift_direction == TeleOp2.LIFT_DIRECTION.DOWN) {
            liftMotor.setTargetPosition(liftTarget);

            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftMotor.setPower(power);
            intakeMotor.setPower(-0.3);


            while (liftMotor.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            liftMotor.setPower(0);
            intakeMotor.setPower(0);
        }

        else{
            liftMotor.setTargetPosition(-liftTarget);

            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftMotor.setPower(power);
            intakeMotor.setPower(0.4);


            while (liftMotor.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            liftMotor.setPower(0);
            intakeMotor.setPower(0);
        }
    }
    public void encoderTurn(double power, double inches, TURN_DIRECTION turn_direction) {
        //Specifications of hardware
        final double WHEEL_DIAMETER_INCHES = 3.77953;
        final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * 3.141592653589793);
        final double GEAR_RATIO = 19.2;
        final double COUNTS_PER_ROTATION_AT_MOTOR = 28;
        final double TICKS_PER_ROTATION = (GEAR_RATIO * COUNTS_PER_ROTATION_AT_MOTOR);
        final double TICKS_PER_INCH = (TICKS_PER_ROTATION) / (WHEEL_CIRCUMFERENCE_INCHES);

        //Target # of ticks for each motor
        int leftFrontTarget;
        int rightFrontTarget;
        int leftBackTarget;
        int rightBackTarget;

        //Resets motor encoders to 0 ticks
        leftFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets the target # of ticks by intaking the number of desired inches of movement and converting to ticks
        leftFrontTarget = leftFrontDriveMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        rightFrontTarget = rightFrontDriveMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        leftBackTarget = leftBackDriveMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        rightBackTarget = rightBackDriveMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);

        if (turn_direction == TURN_DIRECTION.TURN_RIGHT) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(-(leftFrontTarget));
            rightFrontDriveMotor.setTargetPosition(rightFrontTarget);
            leftBackDriveMotor.setTargetPosition(-(leftBackTarget));
            rightBackDriveMotor.setTargetPosition(rightBackTarget);

            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);

            while (leftFrontDriveMotor.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            leftFrontDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);


        }

        if (turn_direction == TURN_DIRECTION.TURN_LEFT) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDriveMotor.setTargetPosition(leftFrontTarget);
            rightFrontDriveMotor.setTargetPosition(-(rightFrontTarget));
            leftBackDriveMotor.setTargetPosition(leftBackTarget);
            rightBackDriveMotor.setTargetPosition(-(rightBackTarget));

            //Tells the motors to drive until they reach the target position
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(power);

            while (leftFrontDriveMotor.isBusy() && opModeIsActive()) {

            }
            //Kills the motors to prepare for next call of method
            leftFrontDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
        }
        leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Kills the motors to prepare for next call of method
        leftFrontDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
        leftBackDriveMotor.setPower(0);
        rightBackDriveMotor.setPower(0);
    }

    public double distanceSensor(SENSOR_DIRECTION sensor_direction) {
        DistanceSensor sensorDistance = null;
        double distance;

        if (sensor_direction == SENSOR_DIRECTION.REAR) {
            sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance_rear");

            Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;


        } else if (sensor_direction == SENSOR_DIRECTION.FRONT) {
            sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance_front");

            Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;


        } else if (sensor_direction == SENSOR_DIRECTION.LEFT) {
            sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance_left");

            Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;


        } else if (sensor_direction == SENSOR_DIRECTION.RIGHT) {
            sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance_right");

            Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        }
        // you can use this as a regular DistanceSensor.
        return sensorDistance.getDistance(DistanceUnit.INCH);



        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.

    }
    public void distSensorDrive(double inputPower, double distanceFromObjectIN, MOVEMENT_DIRECTION movement_direction) {
        if (movement_direction == MOVEMENT_DIRECTION.REVERSE) {
//            do {

            distanceSensor(SENSOR_DIRECTION.REAR);
//            double power = Math.log(distanceSensor(SENSOR_DIRECTION.REAR) - distanceFromObjectCM);
//            if (power > inputPower) {
//                power = inputPower;
//            }

            encoderDrive(inputPower, (distanceSensor(SENSOR_DIRECTION.REAR) - 5), MOVEMENT_DIRECTION.REVERSE);
            encoderDrive(.1, 4, MOVEMENT_DIRECTION.REVERSE);


//                    leftFrontDriveMotor.setPower(-power);
//                    rightFrontDriveMotor.setPower(-power);
//                    leftBackDriveMotor.setPower(-power);
//                    rightBackDriveMotor.setPower(-power);
//                    sleep(200);
//                } while (distanceFromObjectCM > distanceSensor(SENSOR_DIRECTION.REAR));


//            while (distanceFromObjectCM >= distanceSensor(SENSOR_DIRECTION.REAR)) {
//
//                double power = Math.log(distanceSensor(SENSOR_DIRECTION.REAR));
//                if (power > inputPower) {
//                    power = inputPower;
//                }
//                leftFrontDriveMotor.setPower(power);
//                rightFrontDriveMotor.setPower(power);
//                leftBackDriveMotor.setPower(power);
//                rightBackDriveMotor.setPower(power);
//                sleep(50);
//            }
//            if (distanceFromObjectCM <= distanceSensor(SENSOR_DIRECTION.REAR)) {
//                motorKill();
//            }
        }

    }

    public void motorKill(){
        leftFrontDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
        leftBackDriveMotor.setPower(0);
        rightBackDriveMotor.setPower(0);
    }
    public void colorSensorDrive(double power, MOVEMENT_DIRECTION movement_direction, TAPE_COLOR tape_color, COLOR_SENSOR color_sensor) {
        if (tape_color == TAPE_COLOR.RED_TAPE) {
            if (color_sensor == COLOR_SENSOR.LEFT) {
                if (movement_direction == MOVEMENT_DIRECTION.REVERSE) {
                    while (opModeIsActive() && colorSensor(COLOR_SENSOR.LEFT) != 2) {
                        leftFrontDriveMotor.setPower(power);
                        leftBackDriveMotor.setPower(power);
                        rightFrontDriveMotor.setPower(power);
                        rightBackDriveMotor.setPower(power);

                        if (colorSensor(COLOR_SENSOR.LEFT) == 2) {
                            motorKill();
                            encoderDrive(power, .5, MOVEMENT_DIRECTION.FORWARD);
                        }

                    }
                } else if (movement_direction == MOVEMENT_DIRECTION.FORWARD) {
                    while (opModeIsActive() && colorSensor(COLOR_SENSOR.LEFT) != 2) {
                        leftFrontDriveMotor.setPower(-power);
                        leftBackDriveMotor.setPower(-power);
                        rightFrontDriveMotor.setPower(-power);
                        rightBackDriveMotor.setPower(-power);

                        if (colorSensor(COLOR_SENSOR.LEFT) == 2) {
                            motorKill();
                            encoderDrive(power, .5, MOVEMENT_DIRECTION.REVERSE);
                        }
                    }
                } else if (movement_direction == MOVEMENT_DIRECTION.STRAFE_RIGHT) {
                    while (opModeIsActive() && colorSensor(COLOR_SENSOR.LEFT) != 2) {
                        leftFrontDriveMotor.setPower(power);
                        leftBackDriveMotor.setPower(-power);
                        rightFrontDriveMotor.setPower(power);
                        rightBackDriveMotor.setPower(-power);

                        if (colorSensor(COLOR_SENSOR.LEFT) == 2) {
                            motorKill();
                        }
                    }
                } else if (movement_direction == MOVEMENT_DIRECTION.STRAFE_LEFT) {
                    while (opModeIsActive() && colorSensor(COLOR_SENSOR.LEFT) != 2) {
                        leftFrontDriveMotor.setPower(-power);
                        leftBackDriveMotor.setPower(power);
                        rightFrontDriveMotor.setPower(-power);
                        rightBackDriveMotor.setPower(power);

                        if (colorSensor(COLOR_SENSOR.LEFT) == 2) {
                            motorKill();
                        }
                    }
                }
            }
            else{
                if (movement_direction == MOVEMENT_DIRECTION.REVERSE){
                    while (opModeIsActive() && colorSensor(COLOR_SENSOR.RIGHT) != 2) {
                        leftFrontDriveMotor.setPower(power);
                        leftBackDriveMotor.setPower(power);
                        rightFrontDriveMotor.setPower(power);
                        rightBackDriveMotor.setPower(power);

                        if (colorSensor(COLOR_SENSOR.RIGHT) == 2) {
                            motorKill();
                            encoderDrive(power, .5, MOVEMENT_DIRECTION.FORWARD);
                        }
                    }
                } else if (movement_direction == MOVEMENT_DIRECTION.FORWARD) {
                    while (opModeIsActive() && colorSensor(COLOR_SENSOR.RIGHT) != 2) {
                        leftFrontDriveMotor.setPower(-power);
                        leftBackDriveMotor.setPower(-power);
                        rightFrontDriveMotor.setPower(-power);
                        rightBackDriveMotor.setPower(-power);

                        if (colorSensor(COLOR_SENSOR.RIGHT) == 2) {
                            motorKill();
                            encoderDrive(power, .5, MOVEMENT_DIRECTION.REVERSE);
                        }
                    }
                } else if (movement_direction == MOVEMENT_DIRECTION.STRAFE_RIGHT) {
                    while (opModeIsActive() && colorSensor(COLOR_SENSOR.RIGHT) != 2) {
                        leftFrontDriveMotor.setPower(power);
                        leftBackDriveMotor.setPower(-power);
                        rightFrontDriveMotor.setPower(power);
                        rightBackDriveMotor.setPower(-power);

                        if (colorSensor(COLOR_SENSOR.RIGHT) == 2) {
                            motorKill();
                        }
                    }
                } else if (movement_direction == MOVEMENT_DIRECTION.STRAFE_LEFT) {
                    while (opModeIsActive() && colorSensor(COLOR_SENSOR.RIGHT) != 2) {
                        leftFrontDriveMotor.setPower(-power);
                        leftBackDriveMotor.setPower(power);
                        rightFrontDriveMotor.setPower(-power);
                        rightBackDriveMotor.setPower(power);

                        if (colorSensor(COLOR_SENSOR.RIGHT) == 2) {
                            motorKill();
                        }
                    }
                }
            }
        }
        else if (tape_color == TAPE_COLOR.BLUE_TAPE) {
            if (color_sensor == COLOR_SENSOR.LEFT){
                if (movement_direction == MOVEMENT_DIRECTION.REVERSE) {
                    while (opModeIsActive() && colorSensor(COLOR_SENSOR.LEFT) != 1) {
                        leftFrontDriveMotor.setPower(power);
                        leftBackDriveMotor.setPower(power);
                        rightFrontDriveMotor.setPower(power);
                        rightBackDriveMotor.setPower(power);

                        if (colorSensor(COLOR_SENSOR.LEFT) == 1) {
                            motorKill();
                            encoderDrive(power, .5, MOVEMENT_DIRECTION.FORWARD);
                        }
                    }
                } else if (movement_direction == MOVEMENT_DIRECTION.FORWARD) {
                    while (opModeIsActive() && colorSensor(COLOR_SENSOR.LEFT) != 1) {
                        leftFrontDriveMotor.setPower(-power);
                        leftBackDriveMotor.setPower(-power);
                        rightFrontDriveMotor.setPower(-power);
                        rightBackDriveMotor.setPower(-power);

                        if (colorSensor(COLOR_SENSOR.LEFT) == 1) {
                            motorKill();
                            encoderDrive(power, .5, MOVEMENT_DIRECTION.REVERSE);
                        }
                    }
                } else if (movement_direction == MOVEMENT_DIRECTION.STRAFE_RIGHT) {
                    while (opModeIsActive() && colorSensor(COLOR_SENSOR.LEFT) != 1) {
                        leftFrontDriveMotor.setPower(power);
                        leftBackDriveMotor.setPower(-power);
                        rightFrontDriveMotor.setPower(power);
                        rightBackDriveMotor.setPower(-power);

                        if (colorSensor(COLOR_SENSOR.LEFT) == 1) {
                            motorKill();
                        }
                    }
                } else if (movement_direction == MOVEMENT_DIRECTION.STRAFE_LEFT) {
                    while (opModeIsActive() && colorSensor(COLOR_SENSOR.LEFT) != 1) {
                        leftFrontDriveMotor.setPower(-power);
                        leftBackDriveMotor.setPower(power);
                        rightFrontDriveMotor.setPower(-power);
                        rightBackDriveMotor.setPower(power);

                        if (colorSensor(COLOR_SENSOR.LEFT) == 1) {
                            motorKill();
                        }
                    }
                }
            }
            else{
                if (movement_direction == MOVEMENT_DIRECTION.REVERSE) {
                    while (opModeIsActive() && colorSensor(COLOR_SENSOR.RIGHT) != 1) {
                        leftFrontDriveMotor.setPower(power);
                        leftBackDriveMotor.setPower(power);
                        rightFrontDriveMotor.setPower(power);
                        rightBackDriveMotor.setPower(power);

                        if (colorSensor(COLOR_SENSOR.RIGHT) == 1) {
                            motorKill();
                            encoderDrive(power, .5, MOVEMENT_DIRECTION.FORWARD);
                        }
                    }
                } else if (movement_direction == MOVEMENT_DIRECTION.FORWARD) {
                    while (opModeIsActive() && colorSensor(COLOR_SENSOR.RIGHT) != 1) {
                        leftFrontDriveMotor.setPower(-power);
                        leftBackDriveMotor.setPower(-power);
                        rightFrontDriveMotor.setPower(-power);
                        rightBackDriveMotor.setPower(-power);

                        if (colorSensor(COLOR_SENSOR.RIGHT) == 1) {
                            motorKill();
                            encoderDrive(power, .5, MOVEMENT_DIRECTION.REVERSE);
                        }
                    }
                } else if (movement_direction == MOVEMENT_DIRECTION.STRAFE_RIGHT) {
                    while (opModeIsActive() && colorSensor(COLOR_SENSOR.RIGHT) != 1) {
                        leftFrontDriveMotor.setPower(power);
                        leftBackDriveMotor.setPower(-power);
                        rightFrontDriveMotor.setPower(power);
                        rightBackDriveMotor.setPower(-power);

                        if (colorSensor(COLOR_SENSOR.RIGHT) == 1) {
                            motorKill();
                        }
                    }
                } else if (movement_direction == MOVEMENT_DIRECTION.STRAFE_LEFT) {
                    while (opModeIsActive() && colorSensor(COLOR_SENSOR.RIGHT) != 1) {
                        leftFrontDriveMotor.setPower(-power);
                        leftBackDriveMotor.setPower(power);
                        rightFrontDriveMotor.setPower(-power);
                        rightBackDriveMotor.setPower(power);

                        if (colorSensor(COLOR_SENSOR.RIGHT) == 1) {
                            motorKill();
                        }
                    }
                }
            }
        }
    }


    public float colorSensor(COLOR_SENSOR color_sensor) {
        int colorValue = 0;

        View relativeLayout;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
        // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
        // can give very low values (depending on the lighting conditions), which only use a small part
        // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
        // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
        // colors will report at or near 1, and you won't be able to determine what color you are
        // actually looking at. For this reason, it's better to err on the side of a lower gain
        // (but always greater than  or equal to 1).
        float gain = 15;

        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.
        final float[] hsvValuesLeft = new float[3];

        final float[] hsvValuesRight = new float[3];



        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.


        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensorLeft instanceof SwitchableLight) {
            ((SwitchableLight) colorSensorLeft).enableLight(true);
        }

        if (colorSensorRight instanceof SwitchableLight) {
            ((SwitchableLight) colorSensorRight).enableLight(true);
        }

        // Loop until we are asked to stop

        // Show the gain value via telemetry
        //telemetry.addData("Gain", gain);

        // Tell the sensor our desired gain value (normally you would do this during initialization,
        // not during the loop)
        colorSensorLeft.setGain(gain);
        colorSensorRight.setGain(gain);


        // Get the normalized colors from the sensor
        NormalizedRGBA colorsLeft = colorSensorLeft.getNormalizedColors();
        NormalizedRGBA colorsRight = colorSensorRight.getNormalizedColors();

        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for an explanation of HSV color. */

        // Update the hsvValues array by passing it to Color.colorToHSV()

        /* If this color sensor also has a distance sensor, display the measured distance.
         * Note that the reported distance is only useful at very close range, and is impacted by
         * ambient light and surface reflectivity. */
//            if (colorSensor instanceof DistanceSensor) {
//                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
//            }

        if (color_sensor == COLOR_SENSOR.LEFT){
            if (colorsLeft.blue >= .035) {
                colorValue = 1;
            } else if (colorsLeft.red > .017) {
                colorValue = 2;
            }
        }
        else {
            if (colorsRight.blue >= .035) {
                colorValue = 1;
            } else if (colorsRight.red > .017) {
                colorValue = 2;
            }
        }


        return(colorValue);
    }


    public void placeYellowPixel(){
        encoderDrive(0.2, 9, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.2, 2, MOVEMENT_DIRECTION.REVERSE);
        intakeMotor.setPower(0);
        encoderLift(0.25, 4, TeleOp2.LIFT_DIRECTION.UP);
    }
    public int placePurplePixel(boolean blue){
        if (blue){
            sleep(500);
            snapshotAnalysis = pipelineBlue.getAnalysis();
            sleep(500);
            telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
            telemetry.update();

            switch (snapshotAnalysis) {
                case LEFT: {
                    encoderDrive(.7, 33, MOVEMENT_DIRECTION.FORWARD);
                    encoderDrive(0.3, 12, MOVEMENT_DIRECTION.STRAFE_LEFT);
                    placePurplePixelRight.setPosition(0.44);
                    sleep(1200);
                    placePurplePixelRight.setPosition(0.82);
                    return 2;
                }
                case CENTER: {
                    encoderDrive(.7, 39, MOVEMENT_DIRECTION.FORWARD);
                    encoderDrive(0.3, 7, MOVEMENT_DIRECTION.STRAFE_LEFT);
                    placePurplePixelRight.setPosition(0.44);
                    sleep(1200);
                    placePurplePixelRight.setPosition(0.82);

                    return 1;
                }
                case RIGHT: {
                    encoderDrive(.7, 32, MOVEMENT_DIRECTION.FORWARD);
                    placePurplePixelRight.setPosition(0.44);
                    sleep(1200);
                    placePurplePixelRight.setPosition(0.82);


                    return 0;
                }
            }
        }

        else {
            sleep(500);
            snapshotAnalysisRed = pipelineRed.getAnalysis();
            sleep(500);
            telemetry.addData("Snapshot post-START analysis", snapshotAnalysisRed);
            telemetry.update();

            switch (snapshotAnalysisRed) {
                case LEFT: {
                    encoderDrive(.7, 28, MOVEMENT_DIRECTION.FORWARD);
                    encoderDrive(0.3, 3, MOVEMENT_DIRECTION.STRAFE_LEFT);
                    encoderDrive(0.3, 3, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                    placePurplePixelLeft.setPosition(0.5);
                    sleep(1200);
                    placePurplePixelLeft.setPosition(0.1);
                    sleep(200);

                    return 2;
                }
                case CENTER: {
                    encoderDrive(.7, 35, MOVEMENT_DIRECTION.FORWARD);
                    encoderDrive(0.3, 3, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                    placePurplePixelLeft.setPosition(0.5);
                    sleep(200);
                    placePurplePixelLeft.setPosition(0.1);
                    sleep(200);

                    return 1;
                }
                case RIGHT: {
                    encoderDrive(.7, 33, MOVEMENT_DIRECTION.FORWARD);
                    encoderDrive(0.3, 14, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                    placePurplePixelLeft.setPosition(0.5);
                    sleep(1200);
                    placePurplePixelLeft.setPosition(0.1);
                    sleep(200);
                    return 0;
                }
            }
        }
        return 1;
    }


    public int placePurplePixelFar(boolean blue){
        if (blue){
            sleep(500);
            snapshotAnalysis = pipelineBlue.getAnalysis();
            sleep(500);
            telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
            telemetry.update();
            switch (snapshotAnalysis) {
                case LEFT: {
                    encoderDrive(.7, 28, MOVEMENT_DIRECTION.FORWARD);
                    encoderDrive(0.3, 3, MOVEMENT_DIRECTION.STRAFE_LEFT);
                    encoderDrive(0.3, 3, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                    placePurplePixelLeft.setPosition(0.44);
                    sleep(1200);
                    placePurplePixelLeft.setPosition(0.82);

                    return 2;
                }
                case CENTER: {
                    encoderDrive(.7, 30, MOVEMENT_DIRECTION.FORWARD);
                    encoderDrive(0.3, 3, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                    placePurplePixelLeft.setPosition(0.44);
                    sleep(200);
                    placePurplePixelLeft.setPosition(0.82);

                    return 1;
                }
                case RIGHT: {
                    encoderDrive(.7, 33, MOVEMENT_DIRECTION.FORWARD);
                    encoderDrive(0.3, 14, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                    placePurplePixelLeft.setPosition(0.44);
                    sleep(1200);
                    placePurplePixelLeft.setPosition(0.82);

                    return 0;
                }
            }

        }

        else {
            sleep(500);
            snapshotAnalysisRed = pipelineRed.getAnalysis();
            sleep(500);
            telemetry.addData("Snapshot post-START analysis", snapshotAnalysisRed);
            telemetry.update();
            switch (snapshotAnalysisRed) {
                case LEFT: {
                    encoderDrive(.7, 33, MOVEMENT_DIRECTION.FORWARD);
                    encoderDrive(0.3, 14, MOVEMENT_DIRECTION.STRAFE_LEFT);
                    placePurplePixelRight.setPosition(0.5);
                    sleep(1200);
                    placePurplePixelRight.setPosition(0.1);
                    return 2;
                }
                case CENTER: {
                    encoderDrive(.7, 39, MOVEMENT_DIRECTION.FORWARD);
                    encoderDrive(0.3, 7, MOVEMENT_DIRECTION.STRAFE_LEFT);
                    placePurplePixelRight.setPosition(0.5);
                    sleep(1200);
                    placePurplePixelRight.setPosition(0.1);

                    return 1;
                }
                case RIGHT: {
                    encoderDrive(.7, 28, MOVEMENT_DIRECTION.FORWARD);
                    encoderDrive(0.3, 3, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                    encoderDrive(0.3, 3, MOVEMENT_DIRECTION.STRAFE_LEFT);
                    placePurplePixelRight.setPosition(0.5);
                    sleep(1200);
                    placePurplePixelRight.setPosition(0.1);


                    return 0;
                }
            }


        }
        return 1;
    }

    public void blueCloseAuto(boolean parkCorner, boolean parkMiddle){
        int position = placePurplePixel(true);
            if(parkCorner){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 20, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(1, 1, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(1, 25, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 10, MOVEMENT_DIRECTION.STRAFE_LEFT);

            }
            if (parkMiddle){
                encoderDrive(1, 5, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 20, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(1, 1, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(1, 18, MOVEMENT_DIRECTION.FORWARD);
                encoderDrive(1, 10, MOVEMENT_DIRECTION.STRAFE_LEFT);
            }
            sleep(20000);
        }




    public void blueCloseAutoRight(){
        encoderDrive(1.0, 7, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderTurn(0.5, 18.5, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(1.0, 30.5, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.4, 3, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        sleep(500);
        placeYellowPixel();
    }

    public void blueCloseAutoCenter(){
        encoderDrive(1,8, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 17.5, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(1, 32.5, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 2, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        placeYellowPixel();
    }

    public void blueCloseAutoLeft(){
        encoderDrive(1, 10, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 21, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(1, 22, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.4, 6, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        placeYellowPixel();
    }

    public void blueFarAuto(boolean parkCorner, boolean parkMiddle){
        int position = placePurplePixelFar(true);

        if (position == 2){
            blueFarAutoLeft();
            sleep(2000);

        }
        else if (position == 1){
            blueFarAutoCenter();
            sleep(2000);

        }
        else{
            blueFarAutoRight();
            sleep(2000);

            }
        if(parkCorner){
            encoderDrive(1, 20, MOVEMENT_DIRECTION.REVERSE);
            encoderDrive(1, 15, MOVEMENT_DIRECTION.STRAFE_LEFT);

        }
        if (parkMiddle){
            encoderDrive(1, 15, MOVEMENT_DIRECTION.STRAFE_LEFT);
        }
        sleep(20000);
    }


    public void blueFarAutoRight(){
        encoderDrive(0.5, 24, MOVEMENT_DIRECTION.FORWARD);
        sleep(300);
        encoderDrive(0.5, 45, MOVEMENT_DIRECTION.STRAFE_LEFT);
    }

    public void blueFarAutoCenter(){
        encoderDrive(0.5,9, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        encoderDrive(0.5, 22, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 50, MOVEMENT_DIRECTION.STRAFE_LEFT);
    }

    public void blueFarAutoLeft(){
        encoderDrive(0.5, 7, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        encoderDrive(0.5, 22.5, MOVEMENT_DIRECTION.FORWARD);
        sleep(300);
        encoderDrive(0.5, 35, MOVEMENT_DIRECTION.STRAFE_LEFT);

    }

    public void redCloseAuto(boolean parkCorner, boolean parkMiddle){
        int position = placePurplePixel(false);
        if(parkCorner){
            encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
            encoderDrive(1, 20, MOVEMENT_DIRECTION.STRAFE_RIGHT);
            encoderDrive(1, 1, MOVEMENT_DIRECTION.STRAFE_LEFT);
            encoderDrive(1, 25, MOVEMENT_DIRECTION.REVERSE);
            encoderDrive(1, 10, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        }
        if (parkMiddle){
            encoderDrive(1, 5, MOVEMENT_DIRECTION.REVERSE);
            encoderDrive(1, 20, MOVEMENT_DIRECTION.STRAFE_RIGHT);
            encoderDrive(1, 1, MOVEMENT_DIRECTION.STRAFE_LEFT);
            encoderDrive(1, 18, MOVEMENT_DIRECTION.FORWARD);
            encoderDrive(1, 10, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        }
        sleep(20000);
        }



    public void redCloseAutoRight(){
        encoderDrive(1, 10, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 18.5, TURN_DIRECTION.TURN_RIGHT);
        encoderDrive(1, 26,MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 3, MOVEMENT_DIRECTION.STRAFE_LEFT);
        placeYellowPixel();
    }

    public void redCloseAutoCenter(){
        encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        encoderDrive(1,8, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 17.5, TURN_DIRECTION.TURN_RIGHT);
        encoderDrive(1, 29, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 4, MOVEMENT_DIRECTION.STRAFE_LEFT);
        placeYellowPixel();
    }

    public void redCloseAutoLeft(){
        encoderDrive(0.7, 2, MOVEMENT_DIRECTION.REVERSE);
        encoderDrive(1, 7, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        encoderDrive(1, 5, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 18.5, TURN_DIRECTION.TURN_RIGHT);
        encoderDrive(1, 33.5, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 2, MOVEMENT_DIRECTION.STRAFE_LEFT);
        placeYellowPixel();
    }

    public void redFarAuto(boolean parkCorner, boolean parkMiddle){
        int position = placePurplePixelFar(false);

        if (position == 2){
            redFarAutoLeft();

        }
        else if (position == 1){
            redFarAutoCenter();

        }
        else{
            redFarAutoRight();

        }

        if(parkCorner){
            encoderDrive(1, 20, MOVEMENT_DIRECTION.REVERSE);
            encoderDrive(1, 15, MOVEMENT_DIRECTION.STRAFE_RIGHT);

        }
        if (parkMiddle){
            encoderDrive(1, 15, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        }
        sleep(20000);
    }

    public void redFarAutoRight(){
        encoderDrive(0.5, 7, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(0.5, 22.5, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 36, MOVEMENT_DIRECTION.STRAFE_RIGHT);
    }

    public void redFarAutoCenter(){
        encoderDrive(0.5,9, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(0.5, 22, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 50, MOVEMENT_DIRECTION.STRAFE_RIGHT);
    }

    public void redFarAutoLeft(){
        encoderDrive(0.5, 22.5, MOVEMENT_DIRECTION.FORWARD);
        sleep(300);
        encoderDrive(0.5, 42, MOVEMENT_DIRECTION.STRAFE_RIGHT);

    }

    public void declareCameraPropertiesBlue(){
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipelineBlue = new BluePropDetector.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipelineBlue);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public void declareCameraPropertiesRed(){
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipelineRed = new RedPropDetector.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipelineRed);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public void declareHardwareProperties() {
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "frontright");
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "frontleft");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "backright");
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "backleft");
        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor");
        intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");
        placePurplePixelLeft = hardwareMap.get(Servo.class, "purplePixelLeft");
        placePurplePixelRight = hardwareMap.get(Servo.class, "purplePixelRight");

        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "colorSensorLeft");;
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, "colorSensorRight");

        placePurplePixelLeft.setDirection(Servo.Direction.FORWARD);
        placePurplePixelRight.setDirection(Servo.Direction.FORWARD);
        leftFrontDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void declareAutoVariables(){
        if(gamepad1.a&&!aWasPressed) {
            waitTime+=500;
            aWasPressed=true;
        } else if(!gamepad1.a&&aWasPressed) {
            aWasPressed=false;
        }

        if(gamepad1.b&&!bWasPressed&&waitTime>=500) {
            waitTime-=500;
            bWasPressed=true;
        } else if(!gamepad1.b&&bWasPressed) {
            bWasPressed=false;
        }

        if(gamepad1.x&&!xWasPressed) {
            close = true;
            far = false;
            xWasPressed=true;
        } else if(!gamepad1.x&&xWasPressed) {
            xWasPressed=false;
        }

        if(gamepad1.y&&!yWasPressed) {
            far = true;
            close = false;
            yWasPressed=true;
        } else if(!gamepad1.y&&yWasPressed) {
            yWasPressed=false;
        }

        if(gamepad1.right_bumper&&!rBumperWasPressed) {
            parkCorner = true;
            parkMiddle = false;
            rBumperWasPressed=true;
        } else if(!gamepad1.right_bumper&&rBumperWasPressed) {
            rBumperWasPressed=false;
        }

        if(gamepad1.left_bumper&&!lBumperWasPressed) {
            parkCorner = false;
            parkMiddle = true;
            lBumperWasPressed=true;
        } else if(!gamepad1.left_bumper&&lBumperWasPressed) {
            lBumperWasPressed=false;
        }

        telemetry.addData("Wait Duration (A to increase, B to decrease)",waitTime);
        telemetry.addData("Position close (\"X\")", close);
        telemetry.addData("Position far (\"Y\")", far);
        telemetry.addData("Park corner (R Bumper)", parkCorner);
        telemetry.addData("Park middle (L Bumper)", parkMiddle);
    }

    private void initAprilTag() {

        USE_WEBCAM = true;
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }

    public boolean[] getAprilTags()
    {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        boolean[] aprilTags = new boolean[7];
        // Step through the list of detections and display info for each one.
        if (currentDetections.size() != 0)
        {
            for (AprilTagDetection detection : currentDetections)
            {
                if (detection.metadata != null)
                {
                    aprilTags[detection.id] = true;
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                }
                else
                {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }

            }   // end for() loop
        }
        
        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        return aprilTags;
    }// end method telemetryAprilTag()

    public double aprilTagXDistance(int tagNum)
    {
        boolean[] tagLister = getAprilTags();

        if (tagLister[tagNum] == true)
        {
            AprilTagDetection detection;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            return currentDetections.get(tagNum).ftcPose.x;
        }
        else
        {
            return -500;
        }

    }

    enum MOVEMENT_DIRECTION {
        STRAFE_LEFT,
        STRAFE_RIGHT,
        FORWARD,
        REVERSE,
    }

    enum TURN_DIRECTION {
        TURN_LEFT,
        TURN_RIGHT

    }

    enum LIFT_DIRECTION {
        DOWN,
        UP
    }

    enum SENSOR_DIRECTION {
        FRONT,
        REAR,
        LEFT,
        RIGHT

    }

    enum TAPE_COLOR {
        RED_TAPE,
        BLUE_TAPE
    }

    enum COLOR_SENSOR {
        LEFT,
        RIGHT
    }

}

