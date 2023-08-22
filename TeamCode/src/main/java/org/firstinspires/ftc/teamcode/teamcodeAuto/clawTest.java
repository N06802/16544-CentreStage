package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Autonomous
public class clawTest extends LinearOpMode {

    private DcMotorEx turret, lift;
    private Servo arm;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        robot.initTurret();



        arm = hardwareMap.get(Servo.class, "arm");

        arm.scaleRange(0, 0.2);

        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("Lift Pos", lift.getCurrentPosition());
            telemetry.addData("Turret Pos", turret.getCurrentPosition());
            telemetry.addData("Arm Pos", arm.getPosition());
            telemetry.update();
        }

        robot.setArm(3850);
        sleep(3000);
        robot.setArm(0);
        sleep(3000);

        /*
        robot.placeCone(3850, 0);
        sleep(7000);
        robot.placeCone(0,0);*/

    }
}
