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
    boolean bWasPressed = false;
    boolean slowLifter = false;
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
            double lateral =  -gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial - lateral + yaw;
            double rightFrontPower = axial + lateral - yaw;
            double leftBackPower   = axial + lateral + yaw;
            double rightBackPower  = axial - lateral - yaw;
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
                intakePower = 1.0;
            }
            else if (gamepad1.left_bumper == true)
            {
                intakePower = -0.75;
            }
            else
            {
                intakePower = 0;
            }



            if (gamepad1.x == true)
            {
                droneLauncher.setPosition(1.0);
                sleep(200);
                droneLauncher.setPosition(0.9);
                sleep(500);
//                droneLauncher.setPosition(0.5);
//                sleep(200);
            }

            if (gamepad1.b && !bWasPressed) {
                bWasPressed = true;
                if (slowLifter == true)
                {
                    slowLifter = false;
                }
                else
                {
                    slowLifter = true;
                }
            }
            else if (!gamepad1.b && bWasPressed)
            {
                bWasPressed = false;
            }
            if (gamepad1.a == true)
            {
                slowLifter = false;
                intakeMotor.setPower(0);
                encoderDrive(0.2, 9, RobotLinearOpMode.MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(0.2, 2, RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD);
                encoderLift(0.25,5, LIFT_DIRECTION.UP);
                liftMotor.setPower(0);
                sleep(300);
                encoderLift(0.05, 4, LIFT_DIRECTION.DOWN);
                liftMotor.setDirection(DcMotor.Direction.REVERSE);
            }

            if (slowLifter)
            {
                intakePower = -0.5;
            }

            /*leftFrontPower *=0.8;
            rightFrontPower *= 0.75;
            leftBackPower *= 0.95;
            rightBackPower *= 1;
            */
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
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
            intakeMotor.setPower(-0.5);


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

    public void encoderDrive(double power, double inches, RobotLinearOpMode.MOVEMENT_DIRECTION movement_direction) {
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
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets the target # of ticks by intaking the number of desired inches of movement and converting to ticks
        leftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        rightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        leftBackTarget = leftBackDrive.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        rightBackTarget = rightBackDrive.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);

        if (movement_direction == RobotLinearOpMode.MOVEMENT_DIRECTION.FORWARD) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);

            //Tells the motors to drive until they reach the target position
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);

            while (leftFrontDrive.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);


        }

        if (movement_direction == RobotLinearOpMode.MOVEMENT_DIRECTION.REVERSE) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDrive.setTargetPosition(-leftFrontTarget);
            rightFrontDrive.setTargetPosition(-rightFrontTarget);
            leftBackDrive.setTargetPosition(-leftBackTarget);
            rightBackDrive.setTargetPosition(-rightBackTarget);

            //Tells the motors to drive until they reach the target position
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);

            while (leftFrontDrive.isBusy() && opModeIsActive()) {

            }
            //Kills the motors to prepare for next call of method
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }

        if (movement_direction == RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE_RIGHT) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDrive.setTargetPosition(leftFrontTarget * 2);
            rightFrontDrive.setTargetPosition(-rightFrontTarget * 2);
            leftBackDrive.setTargetPosition(-leftBackTarget * 2);
            rightBackDrive.setTargetPosition(rightBackTarget * 2);


            //Tells the motors to drive until they reach the target position
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(.97*power);

            while (leftFrontDrive.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }

        if (movement_direction == RobotLinearOpMode.MOVEMENT_DIRECTION.STRAFE_LEFT) {

            //Sets the target # of ticks to the target position of the motors
            leftFrontDrive.setTargetPosition(-leftFrontTarget * 2);
            rightFrontDrive.setTargetPosition(rightFrontTarget * 2);
            leftBackDrive.setTargetPosition(leftBackTarget * 2);
            rightBackDrive.setTargetPosition(-rightBackTarget * 2);

            //Tells the motors to drive until they reach the target position
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Sets the motor powers to the power entered on use
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(.97 * power);
            rightBackDrive.setPower(power);

            while (leftFrontDrive.isBusy() && opModeIsActive()) {

            }

            //Kills the motors to prepare for next call of method
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Kills the motors to prepare for next call of method
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    enum LIFT_DIRECTION {
        DOWN,
        UP
    }

    enum MOVEMENT_DIRECTION {
        FORWARD,
        REVERSE,
        STRAFE_RIGHT,
        STRAFE_LEFT
    }

}
