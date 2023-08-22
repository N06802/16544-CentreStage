package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Autonomous
public class TestArm extends LinearOpMode {

    private DcMotorEx arm, lift;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift = hardwareMap.get(DcMotorEx.class, "Lift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        arm.setTargetPosition(500);
        lift.setTargetPosition(1700);
        arm.setPower(0.2);
        lift.setPower(0.5);
        while(arm.isBusy()||lift.isBusy()){
            sleep(1);
        }
        arm.setPower(0);
        lift.setPower(0);
        sleep(5000);

        arm.setTargetPosition(0);
        lift.setTargetPosition(0);
        arm.setPower(0.2);
        lift.setPower(0.5);
        while(arm.isBusy()||lift.isBusy()){
            sleep(1);
        }
        arm.setPower(0);
        lift.setPower(0);

    }
}
