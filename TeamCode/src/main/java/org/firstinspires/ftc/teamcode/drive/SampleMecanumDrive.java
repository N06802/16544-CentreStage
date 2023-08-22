package org.firstinspires.ftc.teamcode.drive;

import static android.os.SystemClock.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.StackDetection.StackDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(2, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(10, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.4;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private Servo arm, claw, angle1, angle2, hopper;

    private DcMotorEx leftFront, leftBack, rightBack, rightFront, lift;
    private List<DcMotorEx> motors;
    private DistanceSensor clawDistance, Backdistance;

    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID, new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        /*imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);*/

        //.distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance");
        //Backdistance = hardwareMap.get(DistanceSensor.class, "Backdistance");
        // clawDistance = hardwareMap.get(DistanceSensor.class, "clawDistance");


        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        // lift = hardwareMap.get(DcMotorEx.class, "Lift");

        // arm = hardwareMap.get(Servo.class, "Arm");
        // hopper = hardwareMap.get(Servo.class, "hopper");
        //claw = hardwareMap.get(Servo.class, "Claw");
        //angle1 = hardwareMap.get(Servo.class, "angle1");
        //angle2 = hardwareMap.get(Servo.class, "angle2");

        //arm.setDirection(Servo.Direction.FORWARD);
        // claw.setDirection(Servo.Direction.FORWARD);
        //angle1.setDirection(Servo.Direction.FORWARD);
        //angle2.setDirection(Servo.Direction.FORWARD);
        //hopper.setDirection(Servo.Direction.FORWARD);

        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT, MAX_ANG_VEL, MAX_ANG_ACCEL);
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequenceBuilder(getPoseEstimate()).turn(angle).build());
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequenceBuilder(trajectory.start()).addTrajectory(trajectory).build());
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage());

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY()) + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX()) + VY_WEIGHT * Math.abs(drivePower.getY()) + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(VX_WEIGHT * drivePower.getX(), VY_WEIGHT * drivePower.getY(), OMEGA_WEIGHT * drivePower.getHeading()).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftBack.setPower(v1);
        rightBack.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(maxAngularVel), new MecanumVelocityConstraint(maxVel, trackWidth)));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void rotateAngle(double TGTANGLE, double Precision, double tgtspeed) {
        Orientation angles;
        double ZAngle;
        TGTANGLE = Math.toRadians(TGTANGLE);
        Precision = Math.toRadians(Precision);

        angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        ZAngle = angles.thirdAngle;
        if (ZAngle > TGTANGLE + Precision) {
            leftFront.setPower(-tgtspeed);
            leftBack.setPower(-tgtspeed);
            rightFront.setPower(tgtspeed);
            rightBack.setPower(tgtspeed);
        } else if (ZAngle < TGTANGLE - Precision) {
            leftFront.setPower(tgtspeed);
            leftBack.setPower(tgtspeed);
            rightFront.setPower(-tgtspeed);
            rightBack.setPower(-tgtspeed);
        }
        double angDiff = getAngleDiff(ZAngle, TGTANGLE);
        while (angDiff > 0) {
            angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            ZAngle = angles.thirdAngle;
            angDiff = getAngleDiff(ZAngle, TGTANGLE);
        }

        /*while (ZAngle > TGTANGLE + Precision || ZAngle < TGTANGLE - Precision) {
            ZAngle = angles.thirdAngle;
            angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            if (ZAngle > TGTANGLE + Precision) {
                leftFront.setPower(-tgtspeed);
                leftBack.setPower(-tgtspeed);
                rightFront.setPower(tgtspeed);
                rightBack.setPower(tgtspeed);
            } else if (ZAngle < TGTANGLE - Precision) {
                leftFront.setPower(tgtspeed);
                leftBack.setPower(tgtspeed);
                rightFront.setPower(-tgtspeed);
                rightBack.setPower(-tgtspeed);
            } else {
                break;
            }

            //telemetry.addData("ANGLE: ", ZAngle);
            // telemetry.update();
            // telemetry.update();
        }*/
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void resetIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public double getAngleDiff(double currAngle, double targetAngle) {
        return Math.abs(targetAngle - currAngle);
    }

    public void turretRotate(int rotation, double maxPow) {
        if (rotation > 1200) {
            rotation = 1200;
        } else if (rotation < -1200) {
            rotation = -1200;
        }
    }

    public void setLift(int liftHeight) {
        //lift.setTargetPosition(liftHeight);
        //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // lift.setPower(1);
        while (lift.isBusy()) {
            sleep(1);
        }
    }

    public void setClaw(boolean isClawOpen) {
        if (isClawOpen == true) {
            //   claw.setPosition(1);
        }
        if (isClawOpen == false) {
            // claw.setPosition(0);
        }
        sleep(500);
    }

    public void setArm(double position) {
        //arm.setPosition(position);
    }

    public void placeCone() {
        setArm(0.07);
        sleep(100);
        setLift(-1050);
        dropCone();
        setArm(0.025);
        setLift(0);
    }

    public void placeConeRight() {
        setArm(0.07);
        sleep(100);
        setLift(-1100);
        dropCone();
        setArm(0.025);
        setLift(0);
    }

    public void init2() {
        //angle1.setPosition(0.68);
        //angle2.setPosition(0.32);
        // arm.setPosition(0.09);
        //hopper.setPosition(1);
    }

    public void liftDown() {
        //angle1.setPosition( 0.77);
        // angle2.setPosition(0.23);
    }

    public void dropCone() {
        // hopper.setPosition(0.935);
        sleep(1500);
        // hopper.setPosition(1);
    }

    public void turnSlightly(boolean forward, boolean right, LinearOpMode that) {
        if (forward) {
            if (right) {
                //turn right
                leftFront.setPower(0.4);
                leftBack.setPower(0.4);
                //  while (clawDistance.getDistance(DistanceUnit.INCH) > 9 && that.opModeIsActive()) {
                sleep(1);
                /// }
            } else {
                //turn left
                rightFront.setPower(-0.4);
                rightBack.setPower(-0.4);
                //while (clawDistance.getDistance(DistanceUnit.INCH) > 9 && that.opModeIsActive()) {
                sleep(1);
                // }
            }
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            //move forward to stack
            leftFront.setPower(0.3);
            leftBack.setPower(0.3);
            rightFront.setPower(-0.3);
            rightBack.setPower(-0.3);
            //while(clawDistance.getDistance(DistanceUnit.INCH) > 4 && that.opModeIsActive()){
            sleep(1);
            // }

            leftFront.setPower(-0.2);
            leftBack.setPower(-0.2);
            rightFront.setPower(0.2);
            rightBack.setPower(0.2);
            //while(clawDistance.getDistance(DistanceUnit.INCH) < 4 && that.opModeIsActive()){
            sleep(1);
            //}

            //backwards
        } else {
            leftFront.setPower(-0.2);
            leftBack.setPower(-0.2);
            rightFront.setPower(0.2);
            rightBack.setPower(0.2);
            // while(clawDistance.getDistance(DistanceUnit.INCH) < 4.5 && that.opModeIsActive()){
            sleep(1);
            //}
            if (right) {
                rightBack.setPower(0);
                rightFront.setPower(0);
                leftFront.setPower(-0.4);
                leftBack.setPower(-0.4);
                //     while (Backdistance.getDistance(DistanceUnit.INCH) > 40 && that.opModeIsActive()) {
                sleep(1);
                //     }
                sleep(50);
            } else {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0.4);
                rightBack.setPower(0.4);
                //      while (Backdistance.getDistance(DistanceUnit.INCH) > 40 && that.opModeIsActive()) {
                sleep(1);
                //      }
            }
            leftFront.setPower(-0.2);
            leftBack.setPower(-0.2);
            rightFront.setPower(0.2);
            rightBack.setPower(0.2);
            //  while(Backdistance.getDistance(DistanceUnit.INCH) > 15 && that.opModeIsActive()){
            sleep(1);
            //   }
            leftFront.setPower(0.2);
            leftBack.setPower(0.2);
            rightFront.setPower(-0.2);
            rightBack.setPower(-0.2);
            //  while(Backdistance.getDistance(DistanceUnit.INCH) < 15 && that.opModeIsActive()){
            sleep(1);
            //  }
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void grabCone(double liftHeight, int ms) {
        // arm.setPosition(liftHeight);
        sleep(500);
        goForward(ms);
        setClaw(false);
        sleep(300);
        setArm(0.1);
        sleep(500);
        setClaw(true);
        sleep(300);
        setArm(0.06);
    }

    public void forwardToStack(LinearOpMode that) {
        setArm(0.04);
        // while (clawDistance.getDistance(DistanceUnit.INCH) > 4 && that.opModeIsActive()) {
        leftFront.setPower(0.3);
        leftBack.setPower(0.3);
        rightFront.setPower(-0.3);
        rightBack.setPower(-0.3);
        // }
        //while (clawDistance.getDistance(DistanceUnit.INCH) < 2.5 && that.opModeIsActive()) {
        leftFront.setPower(-0.1);
        leftBack.setPower(-0.1);
        rightFront.setPower(0.1);
        rightBack.setPower(0.1);
        // }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void strafeRightToPole(LinearOpMode that) {
        //distanceArm.setPosition(0.8);
        sleep(1000);
        // while (clawDistance.getDistance(DistanceUnit.CM) > 50 && that.opModeIsActive()) {
        leftFront.setPower(0.3);
        leftBack.setPower(-0.3);
        rightFront.setPower(-0.3);
        rightBack.setPower(0.3);
        //  }
        sleep(150);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(250);
        //while (clawDistance.getDistance(DistanceUnit.CM) > 12.5 && that.opModeIsActive()) {

        leftFront.setPower(0.1);
        leftBack.setPower(0.1);
        rightFront.setPower(0.1);
        rightBack.setPower(0.1);
        //  }
        // while (clawDistance.getDistance(DistanceUnit.CM) < 12 && that.opModeIsActive()) {
        leftFront.setPower(-0.1);
        leftBack.setPower(-0.1);
        rightFront.setPower(-0.1);
        rightBack.setPower(-0.1);
        //  }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        //distanceArm.setPosition(1);
        sleep(500);
    }

    public void strafeLeftToPole(LinearOpMode that) {
        //distanceArm.setPosition(0.8);
        sleep(500);
        // while (clawDistance.getDistance(DistanceUnit.CM) > 50 && that.opModeIsActive()) {
        leftFront.setPower(-0.25);
        leftBack.setPower(0.25);
        rightFront.setPower(0.25);
        rightBack.setPower(-0.25);
        // }
        sleep(150);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(250);
        //while (clawDistance.getDistance(DistanceUnit.CM) > 13 && that.opModeIsActive()) {
        leftFront.setPower(0.08);
        leftBack.setPower(0.1);
        rightFront.setPower(0.1);
        rightBack.setPower(0.1);
        // }
        //while (clawDistance.getDistance(DistanceUnit.CM) < 12.5 && that.opModeIsActive()) {
        leftFront.setPower(-0.1);
        leftBack.setPower(-0.1);
        rightFront.setPower(-0.1);
        rightBack.setPower(-0.1);

        // }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        //distanceArm.setPosition(1);
        sleep(500);
    }

    public void moveBackToPole(LinearOpMode linOp, boolean leftSide) {
        //distanceArm.setPosition(0.8);
        sleep(500);
        // while (clawDistance.getDistance(DistanceUnit.CM) > 40 && linOp.opModeIsActive()) {
        leftFront.setPower(-0.23);
        leftBack.setPower(-0.25);
        rightFront.setPower(-0.25);
        rightBack.setPower(-0.25);
        //}
        if (leftSide) {
            //  while (clawDistance.getDistance(DistanceUnit.CM) > 13 && linOp.opModeIsActive()) {
            leftFront.setPower(0.2);
            leftBack.setPower(-0.2);
            rightFront.setPower(-0.2);
            rightBack.setPower(0.2);
            // }
        } else {
            // while (clawDistance.getDistance(DistanceUnit.CM) > 12.5 && linOp.opModeIsActive()) {
            leftFront.setPower(-0.2);
            leftBack.setPower(0.2);
            rightFront.setPower(0.2);
            rightBack.setPower(-0.2);
            //  }
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        //distanceArm.setPosition(1);
        sleep(500);
    }

    public void strafeForTime(boolean right, int ms) {
        if (right) {
            leftFront.setPower(0.3);
            leftBack.setPower(-0.3);
            rightFront.setPower(-0.3);
            rightBack.setPower(0.3);
        } else {
            leftFront.setPower(-0.3);
            leftBack.setPower(0.3);
            rightFront.setPower(0.3);
            rightBack.setPower(-0.3);
        }
        sleep(ms);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void turnForTime(boolean right, int ms) {
        if (right) {
            leftFront.setPower(0.3);
            leftBack.setPower(0.3);
            rightFront.setPower(0.3);
            rightBack.setPower(0.3);
        } else {
            leftFront.setPower(-0.3);
            leftBack.setPower(-0.3);
            rightFront.setPower(-0.3);
            rightBack.setPower(-0.3);
        }
        sleep(ms);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);


    }

    public void initTurret() {
        //turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // lift.setTargetPositionTolerance(250);
    }

    public void goForward(int ms){
        leftFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightFront.setPower(-0.5);
        rightBack.setPower(-0.5);
        sleep(ms);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void goBackward(int ms){
        leftFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightFront.setPower(0.5);
        rightBack.setPower(0.5);
        sleep(ms);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void turnRight(double pow){
        setMotorPowers(pow, pow, -pow, -pow);
    }

    public void turnLeft(double pow){
        setMotorPowers(-pow, -pow, pow, pow);
    }

}
