package org.firstinspires.ftc.teamcode.auto.R2V2;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.apriltags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.commands.LiftToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_AutoTrajectories;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_CycleToMediumPoleAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.OLD_COMMANDS.R2V2_CycleToPoleAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_MedPoleToStackCloseWaypointAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.OLD_COMMANDS.R2V2_CycleToStackWaypointAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.OLD_COMMANDS.R2V2_DeliverPreloadAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_DeliverPreloadCloseWaypointAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_GrabFromStackCloseWaypointCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.OLD_COMMANDS.R2V2_GrabFromStackCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_StackToMediumPoleAutoCommand;
import org.firstinspires.ftc.teamcode.drive.DriveConstants_R2V2;
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

@Autonomous(group = "drive", name = "\uD83D\uDFE5 R2V2 Red Cycle \uD83D\uDFE5")
public class R2V2_Red_Cycle extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    TrajectorySequence parkTrajectory1, parkTrajectory2, parkTrajectory3, parkTrajectory;
    //private SampleMecanumDrive_R2V2 drive;

    boolean parking = false;

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
    private BatWing batwing;
//    private Camera camera;

//    private R2V2_CycleToPoleAutoCommand cycleToPoleAutoCommand;
//    private R2V2_CycleToStackWaypointAutoCommand cycleToStackWaypointAutoCommand;
//    private R2V2_DeliverPreloadAutoCommand deliverPreloadAutoCommand;
//    private R2V2_GrabFromStackCommand grabFromStackCommand;

    private R2V2_StackToMediumPoleAutoCommand stackToMediumPoleAutoCommand;

    private R2V2_CycleToMediumPoleAutoCommand cycleToMediumPoleAutoCommand;
    //    private R2V2_MedPoleToStackCloseWaypointAutoCommand cycleToStackCloseWaypointAutoCommand;
    private R2V2_DeliverPreloadCloseWaypointAutoCommand deliverPreloadCWAutoCommand;
//    private R2V2_GrabFromStackCloseWaypointCommand grabFromStackCWCommand;

    private R2V2_FollowTrajectoryCommand parkTrajectoryCommand;
//    private LiftToPositionCommand liftToPositionCommand;

    @Override
    public void runOpMode() {
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
        drive = new SampleMecanumDrive_R2V2(hardwareMap);

        // declare subsystems
        drive = new SampleMecanumDrive_R2V2(hardwareMap);
        gripper = new Gripper(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        beaconArm = new BeaconArm(hardwareMap);
        batwing = new BatWing(hardwareMap);
//        camera = new Camera(hardwareMap, telemetry2);

        //generate trajectories, must do this before you declare commands
        drive.setPoseEstimate(R2V2_AutoTrajectories.red_StartPos);
        R2V2_AutoTrajectories.generateTrajectories(drive);

        // declare commands
//        cycleToPoleAutoCommand = new R2V2_CycleToPoleAutoCommand(drive, lift, arm, wrist, gripper, batwing, false);
//        cycleToStackWaypointAutoCommand = new R2V2_CycleToStackWaypointAutoCommand(drive, lift, arm, wrist, gripper, stackIndex, false);
//        deliverPreloadAutoCommand = new R2V2_DeliverPreloadAutoCommand(drive, lift, arm, wrist, gripper, batwing, stackIndex, false);
//        grabFromStackCommand = new R2V2_GrabFromStackCommand(drive, lift, arm, wrist, gripper, batwing, stackIndex, false);


        cycleToMediumPoleAutoCommand = new R2V2_CycleToMediumPoleAutoCommand(drive, lift, arm, wrist, gripper, batwing, false);
        stackToMediumPoleAutoCommand = new R2V2_StackToMediumPoleAutoCommand(drive, lift, arm, wrist, gripper, batwing, false);
//        cycleToStackCloseWaypointAutoCommand = new R2V2_MedPoleToStackCloseWaypointAutoCommand(drive, lift, arm, wrist, gripper, batwing, stackIndex, false);
        deliverPreloadCWAutoCommand = new R2V2_DeliverPreloadCloseWaypointAutoCommand(drive, lift, arm, wrist, gripper, batwing, stackIndex, false);
//        grabFromStackCWCommand = new R2V2_GrabFromStackCloseWaypointCommand(drive, lift, arm, wrist, gripper, batwing, stackIndex, false);
//
//        liftToPositionCommand = new LiftToPositionCommand(lift, 50, 25);

        gripper.close();
////////////////////////////DEFINING PARK TRAJECTORIES//////////////////////////////
        parkTrajectory1 = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.red_StackWaypointPos)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(60, 16, Math.toRadians(270)), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                        SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                .build();

        parkTrajectory2 = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.red_StackWaypointPos)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(36, 16, Math.toRadians(270)), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                        SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                .build();

        parkTrajectory3 = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.red_StackWaypointPos)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(12, 16, Math.toRadians(270)), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                        SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                .build();
        ////////////////////////////////////DONE DEFINING PARK TRAJECTORIES///////////////////////////////////////


        schedule(new SequentialCommandGroup(
                deliverPreloadCWAutoCommand,

                new R2V2_GrabFromStackCloseWaypointCommand(drive, lift, arm, wrist, gripper, batwing, 4, false),
//                cycleToMediumPoleAutoCommand,
                stackToMediumPoleAutoCommand,
                new R2V2_MedPoleToStackCloseWaypointAutoCommand(drive, lift, arm, wrist, gripper, batwing, 3, false),

                new R2V2_GrabFromStackCloseWaypointCommand(drive, lift, arm, wrist, gripper, batwing, 3, false),
//                cycleToMediumPoleAutoCommand,
                stackToMediumPoleAutoCommand,
                new R2V2_MedPoleToStackCloseWaypointAutoCommand(drive, lift, arm, wrist, gripper, batwing, 2, false),

                new R2V2_GrabFromStackCloseWaypointCommand(drive, lift, arm, wrist, gripper, batwing, 2, false),
//                cycleToMediumPoleAutoCommand,
                stackToMediumPoleAutoCommand,
                new R2V2_MedPoleToStackCloseWaypointAutoCommand(drive, lift, arm, wrist, gripper, batwing, 1, false),

                new R2V2_GrabFromStackCloseWaypointCommand(drive, lift, arm, wrist, gripper, batwing, 1, false),
//                cycleToMediumPoleAutoCommand,
                stackToMediumPoleAutoCommand
        ));


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
        ///////////////////////////////END OF INITIALIZATION/////////////////////////////////////////////////////////
/////////////////////////////////////////////////INIT DONE, ASSIGNING APRIL TAG////////////////////////////////////////////
/////////////////////////////////////////////////////////////TAG ASSIGNED, START AUTO LOOP/////////////////////////////////////////////////
        runtime.reset();
        while (!isStopRequested() && opModeIsActive()) {
            if (tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            } else {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            telemetry.addData("lift pos", lift.getLiftPosition());
            telemetry.addData("lift target", lift.getLiftTargetPosition());
            telemetry.addData("lift power", lift.getLiftPower());

            if (runtime.seconds() > 28) {
                if (!parking) {
                    CommandScheduler.getInstance().cancelAll();
                    switch (tagOfInterest.id) {
                        case 1:
                            parkTrajectoryCommand = new R2V2_FollowTrajectoryCommand(drive, parkTrajectory1);
                            telemetry.addLine("park traj 1");
                            break;
                        default:
                        case 2:
                            parkTrajectoryCommand = new R2V2_FollowTrajectoryCommand(drive, parkTrajectory2);
                            telemetry.addLine("park traj 2");
                            break;
                        case 3:
                            parkTrajectoryCommand = new R2V2_FollowTrajectoryCommand(drive, parkTrajectory3);
                            telemetry.addLine("park traj 3");
                            break;
                    }
                    schedule(
                            new ParallelCommandGroup(
                                    new RobotToStateCommand(lift, arm, wrist, gripper, batwing, 100, 0, "autoEnd"),
                                    parkTrajectoryCommand
                            )
                    );
                    parking = true;
                }
                CommandScheduler.getInstance().run();
            } else {
                CommandScheduler.getInstance().run();
            }

        }
        CommandScheduler.getInstance().reset();
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

    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }
}






