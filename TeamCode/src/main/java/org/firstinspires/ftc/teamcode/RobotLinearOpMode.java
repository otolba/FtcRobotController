package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.text.method.MovementMethod;
import android.view.View;

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

import java.util.ArrayList;
import java.util.List;



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

    //Declaration of drive motors
    DcMotor rightFrontDriveMotor;
    DcMotor leftFrontDriveMotor;
    DcMotor rightBackDriveMotor;
    DcMotor leftBackDriveMotor;
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    Servo placePurplePixel;
    Servo placeYellowServo;
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    boolean USE_WEBCAM = false;  // true for webcam, false for phone camera
    boolean placingPixel = false;
    boolean searching;
    boolean aTagSeen = false;
    private ElapsedTime runtime = new ElapsedTime();

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

        if (movement_direction == MOVEMENT_DIRECTION.FORWARD) {

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

        if (movement_direction == MOVEMENT_DIRECTION.REVERSE) {

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

        if (movement_direction == MOVEMENT_DIRECTION.STRAFE_RIGHT) {

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
            leftBackDriveMotor.setPower(power);
            rightBackDriveMotor.setPower(.97*power);

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
            leftBackDriveMotor.setPower(.97 * power);
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

    /*public void encoderLift(double power, double inches, LIFT_DIRECTION lift_direction) {

        //Specifications of hardware
        final double wheelDiameter = 1.5;
        final double wheelCircumference = (wheelDiameter * 3.141592653589793);
        final double ticksPerRotation = 28;
        final double ticksPerInch = (ticksPerRotation / wheelCircumference);

        int liftTarget;


        leftLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftTarget = leftLifter.getCurrentPosition() + (int) (inches * ticksPerInch);


        if (lift_direction == LIFT_DIRECTION.UP) {
            leftLifter.setTargetPosition(liftTarget);
            rightLifter.setTargetPosition(liftTarget);

            leftLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftLifter.setPower(power);
            rightLifter.setPower(power);


            while (leftLifter.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            leftLifter.setPower(0);
            rightLifter.setPower(0);

        }
    }*/

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

    public void blueCloseAutoRight(){
        encoderDrive(0.3, 8, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(0.5, 21, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 9.5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        placePurplePixel.setPosition(1.0);
        sleep(1000);
        placePurplePixel.setPosition(0);
        sleep(1200);
        placePurplePixel.setPosition(0.5);
        sleep(200);
        encoderDrive(0.5, 10, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(0.5, 10, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 15, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(0.5, 15, MOVEMENT_DIRECTION.FORWARD);
        encoderTurn(0.5, 5, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(0.5, 13, MOVEMENT_DIRECTION.FORWARD);

        runtime.reset();

        //encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_LEFT);
        placeYellowServo.setPosition(1.0);
        sleep(1000);
        placeYellowServo.setPosition(0);
        sleep(1200);
        placeYellowServo.setPosition(0.5);
        sleep(200);
    }

    public void blueCloseAutoCenter(){
        encoderDrive(0.5, 29.5, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 5, MOVEMENT_DIRECTION.REVERSE);
        placePurplePixel.setPosition(1.0);
        sleep(1000);
        placePurplePixel.setPosition(0);
        sleep(1200);
        placePurplePixel.setPosition(0.5);
        sleep(200);
        encoderDrive(0.5,10, MOVEMENT_DIRECTION.REVERSE);
        encoderDrive(0.5, 2, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderTurn(0.5, 18.5, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(0.5, 13, MOVEMENT_DIRECTION.FORWARD);
    }

    public void blueCloseAutoCenterPlacePixel(){
        initAprilTag();
        runtime.reset();

        while(aTagSeen == false && runtime.seconds()<3 && opModeIsActive()){
            leftFrontDriveMotor.setPower(0.2);
            rightFrontDriveMotor.setPower(-0.2);
            leftBackDriveMotor.setPower(-0.2);
            rightBackDriveMotor.setPower(0.2);
            if (getAprilTags()[2] == true) {
                aTagSeen = true;
            }
        }
        sleep(1000);
        leftFrontDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
        leftBackDriveMotor.setPower(0);
        rightBackDriveMotor.setPower(0);
        if (aTagSeen)
        {
            encoderDrive(0.5, 20, MOVEMENT_DIRECTION.FORWARD);
            encoderDrive(0.5, 6 , MOVEMENT_DIRECTION.STRAFE_LEFT);
            encoderDrive(0.2, 3, MOVEMENT_DIRECTION.FORWARD);
            sleep(1000);
            placeYellowServo.setPosition(1.0);
            sleep(1000);
            placeYellowServo.setPosition(0);
            sleep(1000);
            placeYellowServo.setPosition(0.5);
            sleep(200);
            encoderDrive(0.5, 5, MOVEMENT_DIRECTION.REVERSE);
            encoderDrive(0.5, 15, MOVEMENT_DIRECTION.STRAFE_LEFT);
        }
        else{
            encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_LEFT);
            encoderDrive(0.5, 20, MOVEMENT_DIRECTION.FORWARD);
            sleep(1000);
            placeYellowServo.setPosition(1.0);
            sleep(1000);
            placeYellowServo.setPosition(0);
            sleep(1000);
            placeYellowServo.setPosition(0.5);
            sleep(200);
            encoderDrive(0.5, 5, MOVEMENT_DIRECTION.REVERSE);
            encoderDrive(0.5, 15, MOVEMENT_DIRECTION.STRAFE_LEFT);
        }
        encoderDrive(0.5, 5, MOVEMENT_DIRECTION.FORWARD);
        placeYellowServo.setPosition(0);
        sleep(1000);
    }

    public void blueCloseAutoLeft(){
        encoderDrive(0.5, 21, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 7, MOVEMENT_DIRECTION.STRAFE_LEFT);
        placePurplePixel.setPosition(1.0);
        sleep(1000);
        placePurplePixel.setPosition(0);
        sleep(1200);
        placePurplePixel.setPosition(0.5);
        sleep(200);
        encoderDrive(0.5, 8, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 14.5, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(0.5, 13, MOVEMENT_DIRECTION.FORWARD);
        encoderTurn(0.5, 6, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(0.5, 13, MOVEMENT_DIRECTION.FORWARD);
        sleep(1000);
        //encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_LEFT);
        placeYellowServo.setPosition(1.0);
        sleep(1000);
        placeYellowServo.setPosition(0);
        sleep(1000);
        placeYellowServo.setPosition(0.5);
        sleep(200);
    }

    public void blueFarAutoRight(){
        encoderDrive(0.3, 8, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(0.5, 21, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 10.5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        placePurplePixel.setPosition(1.0);
        sleep(1000);
        placePurplePixel.setPosition(0);
        sleep(1000);
        placePurplePixel.setPosition(0.5);
        sleep(200);
        encoderDrive(0.5, 10, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(0.5, 10, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 12, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(0.5, 9,MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_LEFT);
    }

    public void blueFarAutoCenter(){
        encoderDrive(0.5, 27.5, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 3, MOVEMENT_DIRECTION.REVERSE);
        placePurplePixel.setPosition(1.0);
        sleep(1000);
        placePurplePixel.setPosition(0);
        sleep(1000);
        placePurplePixel.setPosition(0.5);
        sleep(200);
        encoderDrive(0.5,10, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 12, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(0.5, 5, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_LEFT);
    }

    public void blueFarAutoLeft(){
        encoderDrive(0.5, 21, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 8, MOVEMENT_DIRECTION.STRAFE_LEFT);
        placePurplePixel.setPosition(1.0);
        sleep(1000);
        placePurplePixel.setPosition(0);
        sleep(1000);
        placePurplePixel.setPosition(0.5);
        sleep(200);
        encoderDrive(0.5, 8, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 12, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(0.5, 10, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_LEFT);
    }

    public void redCloseAutoRight(){
        encoderDrive(0.3, 8, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(0.5, 21, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 10.5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        placePurplePixel.setPosition(1.0);
        sleep(1000);
        placePurplePixel.setPosition(0);
        sleep(1000);
        placePurplePixel.setPosition(0.5);
        sleep(200);
        encoderDrive(0.5, 10, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(0.5, 10, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 12, TURN_DIRECTION.TURN_RIGHT);
        encoderDrive(0.5, 15,MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
    }

    public void redCloseAutoCenter(){
        encoderDrive(0.5, 27.5, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 3, MOVEMENT_DIRECTION.REVERSE);
        placePurplePixel.setPosition(1.0);
        sleep(1000);
        placePurplePixel.setPosition(0);
        sleep(1000);
        placePurplePixel.setPosition(0.5);
        sleep(200);
        encoderDrive(0.5,10, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 12, TURN_DIRECTION.TURN_RIGHT);
        encoderDrive(0.5, 10, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
    }

    public void redCloseAutoLeft(){
        encoderDrive(0.5, 21, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 8, MOVEMENT_DIRECTION.STRAFE_LEFT);
        placePurplePixel.setPosition(1.0);
        sleep(1000);
        placePurplePixel.setPosition(0);
        sleep(1000);
        placePurplePixel.setPosition(0.5);
        sleep(200);
        encoderDrive(0.5, 10, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        encoderDrive(0.5, 8, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 12, TURN_DIRECTION.TURN_RIGHT);
        encoderDrive(0.5, 10, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
    }

    public void redFarAutoRight(){
        encoderDrive(0.3, 8, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(0.5, 21, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 10.5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        placePurplePixel.setPosition(1.0);
        sleep(1000);
        placePurplePixel.setPosition(0);
        sleep(1000);
        placePurplePixel.setPosition(0.5);
        sleep(200);
        encoderDrive(0.5, 10, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(0.5, 10, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 12, TURN_DIRECTION.TURN_RIGHT);
        encoderDrive(0.5, 15, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
    }

    public void redFarAutoCenter(){
        encoderDrive(0.5, 27.5, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 3, MOVEMENT_DIRECTION.REVERSE);
        placePurplePixel.setPosition(1.0);
        sleep(1000);
        placePurplePixel.setPosition(0);
        sleep(1000);
        placePurplePixel.setPosition(0.5);
        sleep(200);
        encoderDrive(0.5,10, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 12, TURN_DIRECTION.TURN_RIGHT);
        encoderDrive(0.5, 10, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
    }

    public void redFarAutoLeft(){
        encoderDrive(0.5, 21, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 8, MOVEMENT_DIRECTION.STRAFE_LEFT);
        placePurplePixel.setPosition(1.0);
        sleep(1000);
        placePurplePixel.setPosition(0);
        sleep(1000);
        placePurplePixel.setPosition(0.5);
        sleep(200);
        encoderDrive(0.5, 8, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 12, TURN_DIRECTION.TURN_RIGHT);
        encoderDrive(0.5, 10, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
    }

    public void placePixel(){
        //multiply sensor value by gain
        float gain = 10;

        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value.
        final float[] hsvValues1 = new float[3];
        final float[] hsvValues2 = new float[3];

        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");

//        // If possible, turn the light on in the beginning (it might already be on anyway,
//        // we just make sure it is if we can).
//        if (colorSensor instanceof SwitchableLight) {
//            ((SwitchableLight)colorSensor).enableLight(true);
//        }

        // Wait for the start button to be pressed.
        waitForStart();

        // Loop until we are asked to stop
        while (opModeIsActive()) {
            colorSensor1.setGain(gain);
            colorSensor2.setGain(gain);

            // Get the normalized colors from the sensor
            NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
            NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();

            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(colors1.toColor(), hsvValues1);
            Color.colorToHSV(colors2.toColor(), hsvValues2);

            if (colorSensor1 instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM));
            }
            if (colorSensor2 instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM));
            }

            telemetry.addLine()
                    .addData("Red", "%.3f", colors1.red)
                    .addData("Green", "%.3f", colors1.green)
                    .addData("Blue", "%.3f", colors1.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues1[0])
                    .addData("Saturation", "%.3f", hsvValues1[1])
                    .addData("Value", "%.3f", hsvValues1[2]);
            telemetry.addData("Alpha", "%.3f", colors1.alpha);

            telemetry.addLine()
                    .addData("Red 2", "%.3f", colors2.red)
                    .addData("Green 2", "%.3f", colors2.green)
                    .addData("Blue 2", "%.3f", colors2.blue);
            telemetry.addLine()
                    .addData("Hue 2", "%.3f", hsvValues2[0])
                    .addData("Saturation 2", "%.3f", hsvValues2[1])
                    .addData("Value 2", "%.3f", hsvValues2[2]);
            telemetry.addData("Alpha 2", "%.3f", colors2.alpha);

            //red
            if (hsvValues1[2] > .07){
                telemetry.addData("Color 1 ", "blue");
            }
            //blue
            else if (hsvValues1[2] > .06) {
                telemetry.addData("Color 1 ", "red");
            }

            if (hsvValues2[2] > .07){
                telemetry.addData("Color 1 ", "blue");
            }
            //blue
            else if (hsvValues2[2] > .06) {
                telemetry.addData("Color 1 ", "red");
            }

            if (gamepad1.a){
                placingPixel = true;
            }

            while (placingPixel){
                searching = true;
                leftFrontDriveMotor.setPower(0.2);
                leftBackDriveMotor.setPower(0.2);
                rightFrontDriveMotor.setPower(0.2);
                rightBackDriveMotor.setPower(0.2);
                while (searching) {
                    if (gamepad1.b){
                        searching = false;
                        break;
                    }
                    if (hsvValues1[2] > 0.1) {
                        leftFrontDriveMotor.setPower(0);
                        leftBackDriveMotor.setPower(0);
                    }
                    if (hsvValues2[2] > 0.1) {
                        rightFrontDriveMotor.setPower(0);
                        rightBackDriveMotor.setPower(0);
                    }
                    if (hsvValues1[2] > .1 && hsvValues2[2] > .03) {
                        searching = false;
                    }
                }
                telemetry.addData("Status: ", "Done");
                leftFrontDriveMotor.setPower(0);
                leftBackDriveMotor.setPower(0);
                rightFrontDriveMotor.setPower(0);
                rightBackDriveMotor.setPower(0);
                placingPixel = false;
            }

            telemetry.update();


        }
    }

    public void declareHardwareProperties() {
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "frontright");
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "frontleft");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "backright");
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "backleft");
        placePurplePixel = hardwareMap.get(Servo.class, "purplePixel");
        placeYellowServo = hardwareMap.get(Servo.class, "placeYellowPixel");

        placePurplePixel.setDirection(Servo.Direction.FORWARD);
        leftFrontDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
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

}

