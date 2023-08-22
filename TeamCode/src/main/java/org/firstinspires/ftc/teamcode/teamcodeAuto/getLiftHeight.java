package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class getLiftHeight extends LinearOpMode {

    private DcMotorEx lift;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int liftHeight = 0;

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            if(gamepad1.y){
                liftHeight += 10;
            } else if(gamepad1.a){
                liftHeight -= 10;
            }
            lift.setTargetPosition(liftHeight);
            telemetry.addData("LIFT HEIGHT", liftHeight);
            telemetry.update();
        }
    }
}