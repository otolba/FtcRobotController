package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="singleMotor", group ="Linear OpMode")
public class SingleMotor extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor = null;

    public void runOpMode() {
        DcMotor
        motor  = hardwareMap.get(DcMotor.class, "intake_motor");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        double motorModify = 0.0;
        while (opModeIsActive()){
            if (gamepad1.a == true)
            {
                motorModify -= 0.1;
                sleep(100);
            }
            if (gamepad1.y == true)
            {
                motorModify += 0.1;
                sleep(100);
            }
            double motorPower = gamepad1.b ? motorModify : 0;
            motor.setPower(motorPower);
            telemetry.addData("Motor Power", motorModify);
            telemetry.update();
        }
    }
}
