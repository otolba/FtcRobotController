package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="Testing", group="Linear OpMode")

public class Testing extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftMotor = null;
    private DcMotor intakeMotor = null;
    private DcMotor rightHangMotor = null;
    private Servo droneLauncher = null;
    private Servo placePurplePixelLeft = null;
    private Servo placePurplePixelRight = null;
    private Servo placeYellowPixelLeft = null;
    private Servo placeYellowPixelRight = null;
    private NormalizedColorSensor colorSensorLeft;
    private NormalizedColorSensor colorSensorRight;
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontleft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backleft");
        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor");
        intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");
        placePurplePixelLeft = hardwareMap.get(Servo.class, "purplePixelLeft");
        placePurplePixelRight = hardwareMap.get(Servo.class, "purplePixelRight");
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "colorSensorLeft");;
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, "colorSensorRight");
        placeYellowPixelLeft = hardwareMap.get(Servo.class, "yellowPixelLeft");
        placeYellowPixelRight = hardwareMap.get(Servo.class, "yellowPixelRight");

        placePurplePixelLeft.setDirection(Servo.Direction.FORWARD);
        placePurplePixelRight.setDirection(Servo.Direction.FORWARD);
        placeYellowPixelRight.setDirection(Servo.Direction.FORWARD);
        placeYellowPixelLeft.setDirection(Servo.Direction.FORWARD);
        droneLauncher.setDirection(Servo.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        double purpleLeftPosition = 0;
        double purpleRightPosition = 0;
        double dronePosition = 0;
        double yellowLeftPosition = 0;
        double yellowRightPosition = 0;
        double incrementAmount = 0.001;
        float gain = 25;
        boolean useServos = false;
        while (opModeIsActive()) {
            double leftFrontPower  = 0;
            double rightFrontPower = 0;
            double leftBackPower   = 0;
            double rightBackPower  = 0;
            double liftPower = 0;
            double intakePower = 0;


            if (gamepad1.a)
            {
                rightBackPower = 1.0;
            }
            if (gamepad1.b)
            {
                leftBackPower = 1.0;
            }
            if (gamepad1.x)
            {
                rightFrontPower = 1.0;
            }
            if (gamepad1.y)
            {
                leftFrontPower = 1.0;
            }
            if (gamepad1.left_bumper)
            {
                intakePower = 1.0;
            }
            if (gamepad1.right_bumper)
            {
                intakePower = -1.0;
            }
            if (gamepad1.right_trigger > 0.1)
            {
                liftPower = -1.0;
            }
            if (gamepad1.left_trigger > 0.1)
            {
                liftPower = 1.0;
            }

            if (gamepad2.dpad_left)
            {
                purpleLeftPosition += incrementAmount;
            }
            if (gamepad2.dpad_right)
            {
                purpleLeftPosition -= incrementAmount;
            }
            if (gamepad2.dpad_up)
            {
                purpleRightPosition += incrementAmount;
            }
            if (gamepad2.dpad_down)
            {
                purpleRightPosition -= incrementAmount;
            }
            if (gamepad2.x)
            {
                yellowLeftPosition += incrementAmount;
            }
            if (gamepad2.b)
            {
                yellowLeftPosition -= incrementAmount;
            }
            if (gamepad2.y)
            {
                yellowRightPosition += incrementAmount;
            }
            if (gamepad2.a)
            {
                yellowRightPosition -= incrementAmount;
            }
            if (gamepad2.left_bumper)
            {
                dronePosition += incrementAmount;
            }
            if (gamepad2.right_bumper)
            {
                dronePosition -= incrementAmount;
            }
            if (gamepad2.right_trigger > 0.1)
            {
                gain+= 0.005;
            }
            if (gamepad2.left_trigger > 0.1)
            {
                gain -= 0.005;
            }

            if (gamepad2.left_stick_button)
            {
                useServos = true;
            }
            if (gamepad2.right_stick_button)
            {
                useServos = false;
            }



            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            intakeMotor.setPower(intakePower);
            liftMotor.setPower(liftPower);
            intakeMotor.setPower(intakePower);
            if (useServos)
            {
                placePurplePixelLeft.setPosition(purpleLeftPosition);
                placePurplePixelRight.setPosition(purpleRightPosition);
                placeYellowPixelLeft.setPosition(yellowLeftPosition);
                placeYellowPixelRight.setPosition(yellowRightPosition);
                droneLauncher.setPosition(dronePosition);
            }

            colorSensorLeft.setGain(gain);
            colorSensorRight.setGain(gain);
            final float[] hsvValues = new float[3];
            NormalizedRGBA colors = colorSensorLeft.getNormalizedColors();

            final float[] hsvValuesRight = new float[3];
            NormalizedRGBA colorsRight = colorSensorRight.getNormalizedColors();


            /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
             * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
             * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
             * for an explanation of HSV color. */

            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(colors.toColor(), hsvValues);
            Color.colorToHSV(colorsRight.toColor(), hsvValuesRight);



            telemetry.addData("MOTORS:", "");
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("lift power: ", liftPower);
            telemetry.addData("Intake Power: ", intakePower);
            telemetry.addData("SERVOS: (Click left stick to activate)", "");
            telemetry.addData("Left purple position (left/right)", purpleLeftPosition);
            telemetry.addData("Right purple position (up/down)", purpleRightPosition);
            telemetry.addData("Left yellow position (x/b)", yellowLeftPosition);
            telemetry.addData("Right yellow position (y/a)", yellowRightPosition);
            telemetry.addData("Drone launcher position (left bumper/right bumper)", dronePosition);
            telemetry.addData("SENSORS:", "");
            telemetry.addData("Color sensor gain (right trigger/left trigger)", gain);
            telemetry.addData("Left color sensor: ", "");
            telemetry.addLine()
                    .addData("Left Red", "%.3f", colors.red)
                    .addData("Left Green", "%.3f", colors.green)
                    .addData("Left Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Left Hue", "%.3f", hsvValues[0])
                    .addData("Left Saturation", "%.3f", hsvValues[1])
                    .addData("Left Value", "%.3f", hsvValues[2]);
            telemetry.addData("Left Alpha", "%.3f", colors.alpha);
            telemetry.addData("Right color sensor: ", "");
            telemetry.addLine()
                    .addData("Right Red", "%.3f", colorsRight.red)
                    .addData("Right Green", "%.3f", colorsRight.green)
                    .addData("Right Blue", "%.3f", colorsRight.blue);
            telemetry.addLine()
                    .addData("Right Hue", "%.3f", hsvValuesRight[0])
                    .addData("Right Saturation", "%.3f", hsvValuesRight[1])
                    .addData("Right Value", "%.3f", hsvValuesRight[2]);
            telemetry.addData("Right Alpha", "%.3f", colorsRight.alpha);
            telemetry.update();
        }
    }
}
