package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Autonomous(name = "TurretTest")
public class TurrretTest extends LinearOpMode {

    private DcMotorEx turret = null;
    private DcMotorEx lift = null;
    private Servo arm = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);


        //MAX VALUE 1200
        //MIN VALUE -600
        //MAXPOWER 0.7
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        double turrMaxPow = 0.7;


        //LOWEST VALUE 0
        //LOW POSITION -1780
        //MID POSITION -2700
        //HIGH POSITION -4000
        lift = hardwareMap.get(DcMotorEx.class, "Lift");

        arm = hardwareMap.get(Servo.class, "arm");
        arm.scaleRange(0, 0.2);
        arm.setDirection(Servo.Direction.FORWARD);


        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPositionTolerance(250);


        telemetry.addData("READY", "!");
        telemetry.update();
        waitForStart();

        //LIFT TESTING
        telemetry.addData("POS", "HIGH");
        telemetry.update();
        arm.setPosition(4000.0 / 4500.0);
        lift.setTargetPosition(-4000);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (lift.isBusy()) {
            sleep(1);
        }
        sleep(500);

        telemetry.addData("POS", "MID");
        telemetry.update();
        arm.setPosition(2700.0 / 4500.0);
        lift.setTargetPosition(-2700);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (lift.isBusy()) {
            sleep(1);
        }
        sleep(500);

        telemetry.addData("POS", "LOW");
        telemetry.update();
        arm.setPosition(1780.0 / 4500.0);
        lift.setTargetPosition(-1780);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (lift.isBusy()) {
            sleep(1);
        }
        sleep(500);

        telemetry.addData("POS", "GROUND");
        telemetry.update();
        arm.setPosition(0.0 / 4500.0);
        lift.setTargetPosition(0);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (lift.isBusy()) {
            sleep(1);
        }
        sleep(500);

        telemetry.addData("POS", "MID");
        telemetry.update();
        arm.setPosition(2700.0 / 4500.0);
        lift.setTargetPosition(-2700);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (lift.isBusy()) {
            sleep(1);
        }
        sleep(500);


        telemetry.addData("TURRET", "TESTING");
        telemetry.update();
        //TURRET TESTING
        turret.setTargetPosition(-600);
        turret.setPower(0.5);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (turret.isBusy()) {
            sleep(1);
        }
        sleep(500);

        turret.setTargetPosition(1200);
        turret.setPower(0.5);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (turret.isBusy()) {
            sleep(1);
        }
        sleep(500);

        turret.setTargetPosition(0);
        lift.setTargetPosition(0);
        arm.setPosition(0);
        turret.setPower(0.5);
        lift.setPower(0.4);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (turret.isBusy() || lift.isBusy()) {
            sleep(1);
        }
    }
}
