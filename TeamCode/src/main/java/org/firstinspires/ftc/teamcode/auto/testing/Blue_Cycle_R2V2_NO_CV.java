package org.firstinspires.ftc.teamcode.auto.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Disabled
@Autonomous(group = "drive", name = "R2V2 Blue Cycle NO CV")
public class Blue_Cycle_R2V2_NO_CV extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    final static double fullAutoTime = 30, cycleTime = 5, parkTime = 2;

    boolean finalCycle = false;
    int totalCycles = 0;
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

    TrajectorySequence parkTrajectory;


    FtcDashboard dashboard = FtcDashboard.getInstance();

//    OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

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

//    AprilTagDetection tagOfInterest = null;

    @Override
    public void initialize() {
//        MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // declare subsystems
        drive = new SampleMecanumDrive_R2V2(hardwareMap);
        gripper = new Gripper(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        beaconArm = new BeaconArm(hardwareMap);
        coffin = new BatWing(hardwareMap);
//        camera = new Camera(hardwareMap, telemetry2);

        // declare commands
        cycleToPoleAutoCommand = new R2V2_CycleToPoleAutoCommand(drive, lift, arm, wrist, gripper, coffin, true);
        cycleToStackWaypointAutoCommand = new R2V2_CycleToStackWaypointAutoCommand(drive, lift, arm, wrist, gripper, stackIndex, true);
        deliverPreloadAutoCommand = new R2V2_DeliverPreloadAutoCommand(drive, lift, arm, wrist, gripper, coffin, stackIndex, true);
        grabFromStackCommand = new R2V2_GrabFromStackCommand(drive, lift, arm, wrist, gripper, coffin,  stackIndex, true);
        liftToPositionCommand = new LiftToPositionCommand(lift, 50, 25);

        // declare trajectories
        drive.setPoseEstimate(R2V2_AutoTrajectories.blue_StartPos);
        R2V2_AutoTrajectories.generateTrajectories(drive);

        gripper.close();

        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    telemetry.addData("tag id", tag.id);
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        /*if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        Pose2d startPose = new Pose2d(-36, 66, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence parkTrajectory2;

        switch (tagOfInterest.id) {
            case 1:
                parkTrajectory = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.blue_StackFarWaypointPos)
                        .setReversed(true)
//                        .splineTo(new Vector2d(-12, 12), Math.toRadians(90))
                        //.lineToConstantHeading(new Vector2d(-36, 18))
                        .lineToConstantHeading(new Vector2d(-12, 18))
//                        .splineToConstantHeading(new Vector2d(-12, 12), Math.toRadians(180))
                        .build();

                break;

            case 2:
//                parkTrajectory = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.blue_StackFarWaypointPos)
//                        .setReversed(true)
//                        .splineTo(new Vector2d(-36, 12), Math.toRadians(90))
//                        .build();
                parkTrajectory = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.blue_StackFarWaypointPos)
                        .setReversed(true)
                        .lineToConstantHeading(new Vector2d(-36, 18))
                        .build();
                break;

            case 3:
                parkTrajectory = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.blue_StackFarWaypointPos)
                        .setReversed(true)
//                        .splineTo(new Vector2d(-60, 12), Math.toRadians(90))
                        .lineToConstantHeading(new Vector2d(-58, 15))
                        .build();
                break;

            default:
                parkTrajectory = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.blue_StackFarWaypointPos)
                        .setReversed(true)
                        .lineToConstantHeading(new Vector2d(-36, 18))

                        .build();
                break;
        }

        parkTrajectoryCommand = new R2V2_FollowTrajectoryCommand(drive, parkTrajectory);*/

//    @Override
//    public void run() {
        schedule(new SequentialCommandGroup(
                deliverPreloadAutoCommand,
                grabFromStackCommand,

                new InstantCommand(() -> {
                    stackIndex--;
                }),
                cycleToPoleAutoCommand,
//                liftToPositionCommand,
                cycleToStackWaypointAutoCommand


//                grabFromStackCommand,
//                new InstantCommand(() -> {
//                    stackIndex--;
//                }),
//                cycleToPoleAutoCommand,
//                cycleToStackWaypointAutoCommand,
//
//                new InstantCommand(() -> {
//                    arm.toInitPosition();
////                    lift.setTargetPosition(50);
//                })


//                parkTrajectoryCommand
//                liftToPositionCommand


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

    }


    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}


