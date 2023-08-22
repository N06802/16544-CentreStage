package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware {

    //Create motors
    public DcMotor leftFront  = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack   = null;
    public DcMotor rightBack  = null;

    /* Local OpMode Members */
    HardwareMap hardwareMap = null;
    private ElapsedTime elapsedTime = new ElapsedTime();

    /* Constructor */
    public RobotHardware(){

    }

    /* Initialize Standard Hardware Interfaces */
    public void init(HardwareMap ahwMap){
        /* Save Reference To HardwareMap*/
        hardwareMap = ahwMap;

        //Define and Initialize Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        //Set Motor Direction
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        //Set Motor Power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        //Set ZeroPowerBehaviour
        /* BRAKE
         * The motor stops and then brakes, actively resisting any external force which attempts to turn the motor.
         * FLOAT
         * The motor stops and then floats: an external force attempting to turn the motor is not met with active resistence.
         */
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Set all motors to run using encoders
        //May want to use RUN_WITHOUT_ENCODERS if not using encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //not using encoders for non-drivetrain usage allows for more direct power control
    }
}
