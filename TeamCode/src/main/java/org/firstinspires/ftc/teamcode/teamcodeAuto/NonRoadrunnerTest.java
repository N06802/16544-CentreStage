package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Autonomous
public class NonRoadrunnerTest extends LinearOpMode {

    private DcMotorEx leftFront, leftBack, rightBack, rightFront, turret, lift;
    private DistanceSensor distance1, distance2;
    private Servo arm, claw, distArm;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        lift = hardwareMap.get(DcMotorEx.class, "Lift");
        distance1 = hardwareMap.get(DistanceSensor.class, "Distance");
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        distArm = hardwareMap.get(Servo.class, "distArm");

        double angTravDist;
        initi();

        waitForStart();

        setClaw(false);
        placeCone(3900, 300, true);
        forwardUntilDetect(this, 3200, 30);
        angTravDist = diagonalUntilDetect(this, 25, true);
        sleep(500);
        setClaw(true);
        angleSens(false);
        placeCone(0,0,false);



    }
    private void initi() {
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(Servo.Direction.REVERSE);
        distArm.setDirection(Servo.Direction.REVERSE);
        distArm.scaleRange(0, 0.2);
        arm.scaleRange(0, 0.2);
        claw.scaleRange(0.2, 0.6);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void forwardUntilDetect(LinearOpMode that, double travDist, double poleDist){
        double encodeDist = 0.0;
        while (distance1.getDistance(DistanceUnit.CM )>poleDist && encodeDist < travDist && that.opModeIsActive()){
            setMotorPow(0.5);
            encodeDist = leftFront.getCurrentPosition();
        }
        setMotorPow(0);
    }

    private double diagonalUntilDetect(LinearOpMode that, double poleDist, boolean highPole){
        double encodeDist = 0.0;
        initi();
        if(highPole) {
            angleSens(true);
            sleep(300);
            while (distance2.getDistance(DistanceUnit.CM) > poleDist && that.opModeIsActive()) {
                rightFront.setPower(0.1);
                leftBack.setPower(0.1);
                encodeDist = rightFront.getCurrentPosition();
            }
        } else {
            while(distance1.getDistance(DistanceUnit.CM) > poleDist && that.opModeIsActive()){
                rightFront.setPower(0.1);
                leftBack.setPower(0.1);
                encodeDist = rightFront.getCurrentPosition();
            }
        }
        setMotorPow(0);
        return encodeDist;
    }

    private void setMotorPow(double pow){
        leftFront.setPower(pow);
        leftBack.setPower(pow);
        rightBack.setPower(pow);
        rightFront.setPower(pow);
    }

    public void turretRotate(int rotation, double maxPow) {
        if (rotation > 1200) {
            rotation = 1200;
        } else if (rotation < -600) {
            rotation = -600;
        }
        turret.setTargetPosition(rotation);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(maxPow);
    }

    public void setLift(int liftHeight) {
        if (liftHeight < 0) {
            liftHeight = 0;
        } else if (liftHeight > 4000) {
            liftHeight = 4000;
        }
        liftHeight *= -1;
        lift.setTargetPosition(liftHeight);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
    }

    public void setClaw(boolean isClawOpen) {
        if (isClawOpen == true) {
            claw.setPosition(0.0);
        }
        if (isClawOpen == false) {
            claw.setPosition(1.0);
        }
        sleep(750);
    }

    public void setArm(double liftHeight) {
        double armPos = liftHeight / 4500.0;
        arm.setPosition(armPos);
    }

    public void placeCone(int liftHeight, int rotation, boolean goingUp) {

        if(goingUp) {
            setArm(liftHeight);
            sleep(50);
            setLift(liftHeight);
            sleep(250);
            turretRotate(rotation, 0.5);
        } else {
            turretRotate(rotation, 0.5);
            sleep(250);
            setLift(liftHeight);
            sleep(50);
            setArm(liftHeight);
        }
    }

    private void angleSens(boolean highPole) {
        if(highPole){
            distArm.setPosition(1);
        } else  {
            distArm.setPosition(0);
        }

    }
}

