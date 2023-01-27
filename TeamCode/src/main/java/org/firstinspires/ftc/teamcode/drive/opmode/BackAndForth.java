package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config

//@Disabled
@Autonomous(group = "drive")
public class BackAndForth extends LinearOpMode {

    public static double DISTANCE = 50;
    public double cycles = 0;
    public double numberOfCycles = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive_R2V2 drive = new SampleMecanumDrive_R2V2(hardwareMap);

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(DISTANCE, 0))
                //.forward(DISTANCE)
                .build();

        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
                .lineToConstantHeading(new Vector2d(0, 0))
                //.back(DISTANCE)
                .build();

        Trajectory trajectoryStrafeRight = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();

        Trajectory trajectoryStrafeLeft = drive.trajectoryBuilder(trajectoryStrafeRight.end())
                .strafeLeft(DISTANCE)
                .build();



        waitForStart();

        while (opModeIsActive() && !isStopRequested())/*&& (cycles < numberOfCycles))*/ {
            drive.followTrajectory(trajectoryForward);
            drive.followTrajectory(trajectoryBackward);
//            drive.followTrajectory(trajectoryStrafeRight);
//            drive.followTrajectory(trajectoryStrafeLeft);
           // cycles += 1;
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY()); //
            telemetry.addData("wheel positions", drive.getWheelPositions());
            telemetry.addData("mFL", hardwareMap.get(DcMotorEx.class, "mFL").getCurrentPosition() );
            telemetry.addData("mFR", -hardwareMap.get(DcMotorEx.class, "mFR").getCurrentPosition() );
            telemetry.addData("mBL", hardwareMap.get(DcMotorEx.class, "mBL").getCurrentPosition() );
            telemetry.addData("mBR", hardwareMap.get(DcMotorEx.class, "mBR").getCurrentPosition() );
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}