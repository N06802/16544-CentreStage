package org.firstinspires.ftc.teamcode.teamcodeAuto;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name ="MotorTest")
public class TestMotrs extends LinearOpMode {

    private DcMotorEx leftFront, leftBack, rightBack, rightFront;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");

       leftBack.setDirection(DcMotor.Direction.REVERSE);
       leftFront.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        while(!isStopRequested()) {

            telemetry.addData("LB p", leftBack.getPower());
            telemetry.addData("LF p", leftFront.getPower());
            telemetry.addData("RB p", rightBack.getPower());
            telemetry.addData("RF p", rightFront.getPower());

            telemetry.addData("LB c", leftBack.getCurrent(AMPS));
            telemetry.addData("LF c", leftFront.getCurrent(AMPS));
            telemetry.addData("RB c", rightBack.getCurrent(AMPS));
            telemetry.addData("RF c", rightFront.getCurrent(AMPS));

            //sleep(10000);
            leftBack.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);

            //telemetry.update();

            sleep(100);

            leftBack.setPower(0.5);
            leftFront.setPower(0.5);
            rightBack.setPower(0.5);
            rightFront.setPower(0.5);

            //telemetry.update();

            sleep(1000);

            leftBack.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);

            //telemetry.update();

            sleep(100);

            leftBack.setPower(-0.5);
            leftFront.setPower(-0.5);
            rightBack.setPower(-0.5);
            rightFront.setPower(-0.5);

            //telemetry.update();

            sleep(1000);

            leftBack.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);

            telemetry.update();
        }

    }
}
