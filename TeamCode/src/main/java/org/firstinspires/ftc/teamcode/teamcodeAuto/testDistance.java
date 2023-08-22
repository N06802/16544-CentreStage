package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "testDistance")
public class testDistance extends LinearOpMode {

    private DistanceSensor backDistance;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        backDistance = hardwareMap.get(DistanceSensor.class, "Clawdistance");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("DISTANCE", backDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
