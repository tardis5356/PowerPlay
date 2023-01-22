package org.firstinspires.ftc.teamcode.auto.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.apriltags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous(group = "drive")
public class test_1_5_Barney extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    double fullAutoTime = 30, cycleTime = 5, parkTime = 2;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    AprilTagDetection tagOfInterest = null;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;



    @Override


    public void runOpMode() {
        SampleMecanumDrive_Barney drive = new SampleMecanumDrive_Barney(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //waitForStart();
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    telemetry.addData("tag id", tag.id);
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

//        if (isStopRequested()) return;
         //A2 starting

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        Pose2d startPose = new Pose2d(-36, 66, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence parkTrajectory;

        switch (tagOfInterest.id)
        {
            case 1:
                parkTrajectory = drive.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .splineTo(new Vector2d(-12, 12), Math.toRadians(90))
                        .build();
                break;

            case 2:
                parkTrajectory = drive.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .splineTo(new Vector2d(-36, 12), Math.toRadians(90))
                        .build();
                break;

            case 3:
                parkTrajectory = drive.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .splineTo(new Vector2d(-60, 12), Math.toRadians(90))
                        .build();
                break;

            default:
                parkTrajectory = drive.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .splineTo(new Vector2d(-36, 66), Math.toRadians(90))
                        .build();
                break;
        }


        /*
        scoreX2 - score on x2
        X2ToCones - move from x2 (middle high junction) to a3 (cone stack)
        conesToA2 - strafe from a3 (cone stack) to a2 (next to cones)
        A2ToW3 - forwards from a2 (next to cones) to w3 (high junction)
        W3ToA3 - spline from w3 (high junction) to a3 (cone stack)
        park1
        park2
        park3
         */

            TrajectorySequence deliverPreload = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineTo(new Vector2d(-36, 20), Math.toRadians(270))
                    .splineToLinearHeading(new Pose2d(-32, 8, Math.toRadians(160)), Math.toRadians(340))
                    .setReversed(false)
                    .build();

            TrajectorySequence getFirstCone = drive.trajectorySequenceBuilder(deliverPreload.end())
                    .back(-0.10)
                    .splineToSplineHeading(new Pose2d(-55, 17, Math.toRadians(180)), Math.toRadians(180))
                    .build();

            TrajectorySequence cycle = drive.trajectorySequenceBuilder(getFirstCone.end())
                    .back(0.10)
                    .splineToSplineHeading(new Pose2d(-9, 23, Math.toRadians(212)), Math.toRadians(32))
                    .back(-0.10)
                    .splineToSplineHeading(new Pose2d(-55, 17, Math.toRadians(180)), Math.toRadians(180))
                    .build();
            TrajectorySequence parkZone2 = drive.trajectorySequenceBuilder(cycle.end())
                    .splineToSplineHeading(new Pose2d(-36, 12, Math.toRadians(180)), Math.toRadians(180))
                    .build();


        while (runtime.seconds() < fullAutoTime) {

            drive.followTrajectorySequence(deliverPreload);
            drive.followTrajectorySequence(getFirstCone);

            while (runtime.seconds() < fullAutoTime - cycleTime - parkTime) {

                drive.followTrajectorySequence(cycle);
            }

            drive.followTrajectorySequence(parkTrajectory);
            requestOpModeStop();

        }



    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}

