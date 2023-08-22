package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous(name = "distArmTest")
public class distArmTest extends LinearOpMode {

    private Servo arm, claw, distArm;
    @Override
    public void runOpMode() throws InterruptedException {
        distArm = hardwareMap.get(Servo.class, "distArm");

        waitForStart();

        angleSens(true);
        sleep(1000);
        angleSens(false);
        sleep(1000);
    }

    private void angleSens(boolean highPole) {
        if(highPole){
            distArm.setPosition(1);
        } else  {
            distArm.setPosition(0);
        }

    }
}
