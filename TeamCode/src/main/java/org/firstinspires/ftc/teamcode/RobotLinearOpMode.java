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

    //Declaration of drive motors
    DcMotor rightFrontDriveMotor;
    DcMotor leftFrontDriveMotor;
    DcMotor rightBackDriveMotor;
    DcMotor leftBackDriveMotor;
    DcMotor liftMotor;
    DcMotor intakeMotor;
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    Servo placePurplePixel;
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
            intakeMotor.setPower(0.2);


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

    public void placeYellowPixel(){
        encoderLift(0.2, 4, TeleOp2.LIFT_DIRECTION.UP);
        liftMotor.setPower(0);
        sleep(300);
        encoderLift(0.05, 4, TeleOp2.LIFT_DIRECTION.DOWN);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    public int placePurplePixel(){
        encoderDrive(1, 1.5, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(1, 2, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        intakeMotor.setPower(0.4);
        sleep(500);
        snapshotAnalysis = pipelineBlue.getAnalysis();
        sleep(500);
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        switch (snapshotAnalysis){
            case RIGHT:{
                encoderDrive(1.0, 3, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(1.0, 18, MOVEMENT_DIRECTION.FORWARD);
                encoderDrive(1.0, 8.5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                placePurplePixel.setPosition(1.0);
                sleep(200);
                placePurplePixel.setPosition(0);
                sleep(1200);
                placePurplePixel.setPosition(0.5);
                sleep(200);
                return 0;
            }
        }

        encoderDrive(1, 1, MOVEMENT_DIRECTION.STRAFE_LEFT);
        sleep(500);
        snapshotAnalysis = pipelineBlue.getAnalysis();
        sleep(500);
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        switch (snapshotAnalysis){
            case LEFT:{
                encoderDrive(1, 15, MOVEMENT_DIRECTION.FORWARD);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.STRAFE_LEFT);
                placePurplePixel.setPosition(1.0);
                sleep(200);
                placePurplePixel.setPosition(0);
                sleep(1200);
                placePurplePixel.setPosition(0.5);
                sleep(200);
                return 2;
            }
            case CENTER:{
                encoderDrive(1, 22, MOVEMENT_DIRECTION.FORWARD);
                encoderDrive(0.5, 3, MOVEMENT_DIRECTION.REVERSE);
                placePurplePixel.setPosition(1.0);
                sleep(200);
                placePurplePixel.setPosition(0);
                sleep(1200);
                placePurplePixel.setPosition(0.5);
                sleep(200);
                return 1;
            }
            case RIGHT:{
                encoderDrive(1, 22, MOVEMENT_DIRECTION.FORWARD);
                encoderDrive(0.5, 3, MOVEMENT_DIRECTION.REVERSE);
                placePurplePixel.setPosition(1.0);
                sleep(200);
                placePurplePixel.setPosition(0);
                sleep(1200);
                placePurplePixel.setPosition(0.5);
                sleep(200);
                return 1;
            }
        }
        return 1;
    }
    public void blueCloseAuto(boolean parkCorner, boolean parkMiddle){
        int position = placePurplePixel();

        if (position == 2){
            blueCloseAutoLeft();
            intakeMotor.setPower(0);
            sleep(2000);
            if(parkCorner){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 15, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.FORWARD);
            }
            if (parkMiddle){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 10, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(1, 7, MOVEMENT_DIRECTION.FORWARD);
            }
            sleep(20000);
        }
        else if (position == 1){
            blueCloseAutoCenter();
            intakeMotor.setPower(0);
            sleep(2000);
            if(parkCorner){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 18, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.FORWARD);
            }
            if (parkMiddle){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 7, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(1, 7, MOVEMENT_DIRECTION.FORWARD);
            }
            sleep(20000);
        }
        else{
            blueCloseAutoRight();
            intakeMotor.setPower(0);
            sleep(2000);
            if(parkCorner){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 20, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.FORWARD);
            }
            if (parkMiddle){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(1, 7, MOVEMENT_DIRECTION.FORWARD);
            }
            sleep(20000);
        }
    }

    public void blueCloseAutoRight(){
        encoderDrive(1.0, 7, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(1.0, 5, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 18.5, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(1.0, 31, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(1.0, 6, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        encoderDrive(0.3, 2, MOVEMENT_DIRECTION.FORWARD);
        sleep(500);
        placeYellowPixel();
    }

    public void blueCloseAutoCenter(){
        encoderDrive(1,5, MOVEMENT_DIRECTION.REVERSE);
        encoderDrive(1, 4, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderTurn(0.5, 18.5, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(0.5, 31, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(1.0, 4, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(0.3, 2, MOVEMENT_DIRECTION.FORWARD);
        sleep(500);
        placeYellowPixel();
    }

    public void blueCloseAutoLeft(){
        encoderDrive(1, 5, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 18.5, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(1, 20, MOVEMENT_DIRECTION.FORWARD);
        placeYellowPixel();
    }

    public void blueFarAuto(boolean parkCorner, boolean parkMiddle){
        int position = placePurplePixel();

        if (position == 2){
            blueFarAutoLeft();
            sleep(2000);
            if(parkCorner){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 10, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.FORWARD);
            }
            if (parkMiddle){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 15, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(1, 7, MOVEMENT_DIRECTION.FORWARD);
            }
            sleep(20000);
        }
        else if (position == 1){
            blueFarAutoCenter();
            sleep(2000);
            if(parkCorner){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 15, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.FORWARD);
            }
            if (parkMiddle){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 10, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(1, 7, MOVEMENT_DIRECTION.FORWARD);
            }
            sleep(20000);
        }
        else{
            blueFarAutoRight();
            sleep(2000);
            if (parkCorner) {
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 20, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.FORWARD);
            }
            if (parkMiddle) {
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(1, 7, MOVEMENT_DIRECTION.FORWARD);
            }
            sleep(20000);
        }
    }

    public void blueFarAutoRight(){
        encoderDrive(1, 2, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        encoderDrive(1, 3, MOVEMENT_DIRECTION.FORWARD);
        encoderTurn(0.5, 18.5, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(1, 55, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(1, 3, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(0.3, 2, MOVEMENT_DIRECTION.FORWARD);
        sleep(500);
        placeYellowPixel();
    }

    public void blueFarAutoCenter(){
        encoderDrive(1,3, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        encoderDrive(1, 10, MOVEMENT_DIRECTION.FORWARD);
        encoderTurn(0.5, 18.5, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(1, 50, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(1, 6, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(0.3, 3, MOVEMENT_DIRECTION.FORWARD);
        placeYellowPixel();
    }

    public void blueFarAutoLeft(){
        encoderDrive(1, 4, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        encoderDrive(1, 15, MOVEMENT_DIRECTION.FORWARD);
        encoderTurn(0.5, 18.5, TURN_DIRECTION.TURN_LEFT);
        encoderDrive(1, 45, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(1, 9, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(0.3, 3, MOVEMENT_DIRECTION.FORWARD);
        placeYellowPixel();
    }

    public void redCloseAuto(boolean parkCorner, boolean parkMiddle){
        int position = placePurplePixel();

        if (position == 2){
            redCloseAutoLeft();
            sleep(2000);
            if(parkCorner){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 10, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.FORWARD);
            }
            if (parkMiddle){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 15, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(1, 7, MOVEMENT_DIRECTION.FORWARD);
            }
            sleep(20000);
        }
        else if (position == 1){
            redCloseAutoCenter();
            sleep(2000);
            if(parkCorner){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 18, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.FORWARD);
            }
            if (parkMiddle){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 7, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(1, 7, MOVEMENT_DIRECTION.FORWARD);
            }
            sleep(20000);
        }
        else{
            redCloseAutoRight();
            sleep(2000);
            if(parkCorner){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 20, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.FORWARD);
            }
            if (parkMiddle){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(1, 7, MOVEMENT_DIRECTION.FORWARD);
            }
            sleep(20000);
        }
    }

    public void redCloseAutoRight(){
        encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
        encoderTurn(0.5, 18.5, TURN_DIRECTION.TURN_RIGHT);
        encoderDrive(1, 14,MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(1, 2, MOVEMENT_DIRECTION.STRAFE_LEFT);
        placeYellowPixel();
    }

    public void redCloseAutoCenter(){
        encoderDrive(1,4, MOVEMENT_DIRECTION.REVERSE);
        encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        encoderTurn(0.5, 18.5, TURN_DIRECTION.TURN_RIGHT);
        encoderDrive(1, 16, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(1, 2, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        placeYellowPixel();
    }

    public void redCloseAutoLeft(){
        encoderDrive(1, 5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        encoderTurn(0.5, 18.5, TURN_DIRECTION.TURN_RIGHT);
        encoderDrive(1, 16, MOVEMENT_DIRECTION.FORWARD);
        placeYellowPixel();
    }

    public void redFarAuto(boolean parkCorner, boolean parkMiddle){
        int position = placePurplePixel();

        if (position == 2){
            redFarAutoLeft();
            sleep(2000);
            if(parkCorner){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 10, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.FORWARD);
            }
            if (parkMiddle){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 15, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(1, 7, MOVEMENT_DIRECTION.FORWARD);
            }
            sleep(20000);
        }
        else if (position == 1){
            redFarAutoCenter();
            sleep(2000);
            if(parkCorner){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 18, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.FORWARD);
            }
            if (parkMiddle){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 7, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(1, 7, MOVEMENT_DIRECTION.FORWARD);
            }
            sleep(20000);
        }
        else{
            redFarAutoRight();
            sleep(2000);
            if(parkCorner){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 20, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.FORWARD);
            }
            if (parkMiddle){
                encoderDrive(1, 3, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(1, 5, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(1, 7, MOVEMENT_DIRECTION.FORWARD);
            }
            sleep(20000);
        }
    }

    public void redFarAutoRight(){
        encoderDrive(1, 5, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(1, 18, MOVEMENT_DIRECTION.FORWARD);
        encoderTurn(0.5, 18.5, TURN_DIRECTION.TURN_RIGHT);
        encoderDrive(1, 38, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(1, 15, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        encoderDrive(0.3, 3, MOVEMENT_DIRECTION.FORWARD);
        placeYellowPixel();
    }

    public void redFarAutoCenter(){
        encoderDrive(1,5, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(1, 18, MOVEMENT_DIRECTION.FORWARD);
        encoderTurn(0.5, 18.5, TURN_DIRECTION.TURN_RIGHT);
        encoderDrive(1, 38, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(1, 12, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        encoderDrive(0.3, 3, MOVEMENT_DIRECTION.FORWARD);
        placeYellowPixel();
    }

    public void redFarAutoLeft(){
        encoderDrive(1, 5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        encoderDrive(1, 18, MOVEMENT_DIRECTION.FORWARD);
        encoderTurn(0.5, 18.5, TURN_DIRECTION.TURN_RIGHT);
        encoderDrive(0.5, 38, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(1, 10, MOVEMENT_DIRECTION.STRAFE_RIGHT);
        encoderDrive(0.3, 3, MOVEMENT_DIRECTION.FORWARD);
        placeYellowPixel();
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
        placePurplePixel = hardwareMap.get(Servo.class, "purplePixel");
        //placeYellowServo = hardwareMap.get(Servo.class, "placeYellowPixel");

        placePurplePixel.setDirection(Servo.Direction.FORWARD);
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

}

