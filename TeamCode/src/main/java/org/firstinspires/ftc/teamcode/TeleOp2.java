package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="TeleOp", group="Linear OpMode")
public class TeleOp2 extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftMotor = null;
    private DcMotor intakeMotor = null;
    private Servo droneLauncher = null;

    private int liftState = 0;
    boolean intakePower = false;
    double servoPower = 0;
    boolean lifting = false;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontleft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");
        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor");
        intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
//        yellowPlacer.setPosition(1.0);
//        sleep(1000);
//        yellowPlacer.setPosition(0);
//        sleep(500);
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            double liftPower = 0;
            double intakePower = 0;


            // Send calculated power to wheels
            /*if (leftFrontPower <= -.05){
                leftFrontPower += -.12;
            }
            if (leftBackPower >= .05){
                leftBackPower += .12;
            }
            if (rightFrontPower >= .05){
                rightFrontPower += .12;
            }
            if (rightBackPower >= .05){
                rightBackPower += .12;
            }*/
            liftPower += gamepad1.right_trigger*-1;
            liftPower += gamepad1.left_trigger;

            if (gamepad1.right_bumper == true)
            {
                intakePower = 0.75;
            }
            else if (gamepad1.left_bumper == true)
            {
                intakePower = -1.0;
            }
            else
            {
                intakePower = 0;
            }

            if (gamepad1.a == true)
            {
                encoderLift(0.25, 3.3, LIFT_DIRECTION.UP);
                liftMotor.setPower(0);
                sleep(300);
//                encoderLift(0.1, 2, LIFT_DIRECTION.UP);
//
//                encoderLift(0.2, 1, LIFT_DIRECTION.DOWN);
//                liftMotor.setDirection(DcMotor.Direction.REVERSE);
            }
            if (gamepad1.b == true)
            {
                liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                liftMotor.setPower(0);
            }

            if (gamepad1.x == true)
            {
                droneLauncher.setPosition(servoPower);
                servoPower -= 0.1;
            }



            leftFrontDrive.setPower(leftFrontPower*0.9);
            rightFrontDrive.setPower(rightFrontPower*0.9);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            intakeMotor.setPower(intakePower);

            liftMotor.setPower(liftPower);






            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("LeftTrigger RightTrigger","%4.2f, %4.2f", gamepad1.left_trigger, gamepad1.right_trigger);
            telemetry.addData("lift power", liftPower);
            telemetry.addData("SERVO POWER", servoPower);
            telemetry.update();
        }
    }


    public void encoderLift(double power, double inches, LIFT_DIRECTION lift_direction) {

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


        if (lift_direction == LIFT_DIRECTION.UP) {
            liftMotor.setTargetPosition(liftTarget);

            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftMotor.setPower(power);
            intakeMotor.setPower(-0.2);


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

    enum LIFT_DIRECTION {
        DOWN,
        UP
    }
}
