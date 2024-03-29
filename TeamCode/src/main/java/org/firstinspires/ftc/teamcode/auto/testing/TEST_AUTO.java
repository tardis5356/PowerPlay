package org.firstinspires.ftc.teamcode.auto.testing;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.apriltags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.commands.LiftToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_AutoTrajectories;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.OLD_COMMANDS.R2V2_CycleToPoleAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.OLD_COMMANDS.R2V2_CycleToStackWaypointAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.OLD_COMMANDS.R2V2_DeliverPreloadAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.OLD_COMMANDS.R2V2_GrabFromStackCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.BeaconArm;
import org.firstinspires.ftc.teamcode.subsystems.BatWing;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous(group = "drive", name = "TEST")
public class TEST_AUTO extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    TrajectorySequence parkTrajectory;
    //private SampleMecanumDrive_R2V2 drive;



    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    int stackIndex = 4;

    private SampleMecanumDrive_R2V2 drive;
    private Lift lift;
    private Arm arm;
    private Wrist wrist;
    private Gripper gripper;
    private BeaconArm beaconArm;
    private BatWing coffin;
//    private Camera camera;

    private R2V2_CycleToPoleAutoCommand cycleToPoleAutoCommand;
    private R2V2_CycleToStackWaypointAutoCommand cycleToStackWaypointAutoCommand;
    private R2V2_DeliverPreloadAutoCommand deliverPreloadAutoCommand;
    private R2V2_GrabFromStackCommand grabFromStackCommand;
    private R2V2_FollowTrajectoryCommand parkTrajectoryCommand;
    private LiftToPositionCommand liftToPositionCommand;

    @Override
    public void runOpMode(){
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
        drive = new SampleMecanumDrive_R2V2(hardwareMap);

        // declare subsystems
        drive = new SampleMecanumDrive_R2V2(hardwareMap);
        gripper = new Gripper(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        beaconArm = new BeaconArm(hardwareMap);
        coffin = new BatWing(hardwareMap);
//        camera = new Camera(hardwareMap, telemetry2);

        //generate trajectories, must do this before you declare commands
        drive.setPoseEstimate(R2V2_AutoTrajectories.red_StartPos);
        R2V2_AutoTrajectories.generateTrajectories(drive);

        // declare commands
        cycleToPoleAutoCommand = new R2V2_CycleToPoleAutoCommand(drive, lift, arm, wrist, gripper, coffin, false);
        cycleToStackWaypointAutoCommand = new R2V2_CycleToStackWaypointAutoCommand(drive, lift, arm, wrist, gripper, stackIndex, false);
        deliverPreloadAutoCommand = new R2V2_DeliverPreloadAutoCommand(drive, lift, arm, wrist, gripper, coffin, stackIndex, false);
        grabFromStackCommand = new R2V2_GrabFromStackCommand(drive, lift, arm, wrist, gripper, coffin, stackIndex, false);
        liftToPositionCommand = new LiftToPositionCommand(lift, 50, 25);

        gripper.close();
        schedule(new SequentialCommandGroup(

                deliverPreloadAutoCommand
//                grabFromStackCommand,
//
//                new InstantCommand(() -> {
//                    stackIndex--;
//                }),
//                cycleToPoleAutoCommand,
//                cycleToStackWaypointAutoCommand,
////                grabFromStackCommand,
////                new InstantCommand(() -> {
////                    stackIndex--;
////                }),
////                cycleToPoleAutoCommand,
////                cycleToStackWaypointAutoCommand,
//
//                new InstantCommand(() -> {
//                    arm.toInitPosition();
////                    lift.setTargetPosition(50);
//                }),
//                liftToPositionCommand,
//                parkTrajectoryCommand

                //grabFromStackCommand,

//                new InstantCommand(() -> {
//                    stackIndex--;
//                }),
//                cycleToPoleAutoCommand,
//                cycleToStackWaypointAutoCommand//, grabFromStackCommand,
//                new InstantCommand(() -> { stackIndex--; }),
//                cycleToPoleAutoCommand,
//                cycleToStackWaypointAutoCommand, grabFromStackCommand,
//                new InstantCommand(() -> { stackIndex--; }),
//                cycleToPoleAutoCommand
        ));


        while (!isStarted() && !isStopRequested()){


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
        ///////////////////////////////END OF INITIALIZATION/////////////////////////////////////////////////////////
/////////////////////////////////////////////////INIT DONE, ASSIGNING APRIL TAG////////////////////////////////////////////
        switch (tagOfInterest.id) {
            case 1:
                parkTrajectory = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.red_StackFarWaypointPos)
                        .setReversed(true)
//                        .splineTo(new Vector2d(-12, 12), Math.toRadians(90))
                        //.lineToConstantHeading(new Vector2d(-36, 18))
                        .lineToConstantHeading(new Vector2d(58, 15))
//                        .splineToConstantHeading(new Vector2d(-12, 12), Math.toRadians(180))
                        .build();

                break;

            case 2:
//                parkTrajectory = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.blue_StackFarWaypointPos)
//                        .setReversed(true)
//                        .splineTo(new Vector2d(-36, 12), Math.toRadians(90))
//                        .build();
                parkTrajectory = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.red_StackFarWaypointPos)
                        .setReversed(true)
                        .lineToConstantHeading(new Vector2d(36, 18))
                        .build();
                break;

            case 3:
                parkTrajectory = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.red_StackFarWaypointPos)
                        .setReversed(true)
//                        .splineTo(new Vector2d(-60, 12), Math.toRadians(90))
                        .lineToConstantHeading(new Vector2d(12, 18))
                        .build();
                break;

            default:
                parkTrajectory = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.red_StackFarWaypointPos)
                        .setReversed(true)
                        .lineToConstantHeading(new Vector2d(36, 18))

                        .build();
                break;
        }

        parkTrajectoryCommand = new R2V2_FollowTrajectoryCommand(drive, parkTrajectory);
/////////////////////////////////////////////////////////////TAG ASSIGNED, START AUTO LOOP/////////////////////////////////////////////////
        while (!isStopRequested() && opModeIsActive()) {
            if (tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            } else {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            CommandScheduler.getInstance().run();

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

    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }






//    ElapsedTime runtime = new ElapsedTime();
//
//    final static double fullAutoTime = 30, cycleTime = 5, parkTime = 2;
//
//    boolean finalCycle = false;
//    int totalCycles = 0;
//    int stackIndex = 4;
//
//    private SampleMecanumDrive_R2V2 drive;
//    private Lift lift;
//    private Arm arm;
//    private Wrist wrist;
//    private Gripper gripper;
//    private BeaconArm beaconArm;
//    private Coffin coffin;
////    private Camera camera;
//
//    private R2V2_CycleToPoleAutoCommand cycleToPoleAutoCommand;
//    private R2V2_CycleToStackWaypointAutoCommand cycleToStackWaypointAutoCommand;
//    private R2V2_DeliverPreloadAutoCommand deliverPreloadAutoCommand;
//    private R2V2_GrabFromStackCommand grabFromStackCommand;
//    private R2V2_FollowTrajectoryCommand parkTrajectoryCommand;
//    private LiftToPositionCommand liftToPositionCommand;
//
//    TrajectorySequence parkTrajectory;
//
//
//
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//
//    OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//
//    static final double FEET_PER_METER = 3.28084;
//
//    // Lens intrinsics
//    // UNITS ARE PIXELS
//    // NOTE: this calibration is for the C920 webcam at 800x448.
//    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//
//    // UNITS ARE METERS
//    double tagsize = 0.166;
//
//    // Tag ID 1,2,3 from the 36h11 family
//    int LEFT = 1;
//    int MIDDLE = 2;
//    int RIGHT = 3;
//
//    AprilTagDetection tagOfInterest = null;



    }






