package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
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

        placePurplePixelLeft.setDirection(Servo.Direction.FORWARD);
        placePurplePixelRight.setDirection(Servo.Direction.FORWARD);
        droneLauncher.setDirection(Servo.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            double leftFrontPower  = 0;
            double rightFrontPower = 0;
            double leftBackPower   = 0;
            double rightBackPower  = 0;
            double liftPower = 0;
            double intakePower = 0;
            double purpleLeftPosition = 0;
            double purpleRightPosition = 0;
            double dronePosition = 0;
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
            if (gamepad1.dpad_left)
            {
                purpleLeftPosition += 0.1;
            }
            if (gamepad1.dpad_left)
            {
                purpleLeftPosition += 0.1;
            }



            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            intakeMotor.setPower(intakePower);
            liftMotor.setPower(liftPower);
            intakeMotor.setPower(intakePower);
            placePurplePixelLeft.setPosition(purpleLeftPosition);
            placePurplePixelRight.setPosition(purpleRightPosition);
            droneLauncher.setPosition(dronePosition);
        }
    }
}
