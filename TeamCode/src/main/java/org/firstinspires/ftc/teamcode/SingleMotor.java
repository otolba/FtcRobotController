package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        while (opModeIsActive()){
            double motorPower = gamepad1.b ? 0.5 : 0;
        }
    }
}
