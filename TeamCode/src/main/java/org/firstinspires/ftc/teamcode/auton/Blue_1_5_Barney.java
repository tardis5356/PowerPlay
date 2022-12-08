package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.LiftToIntakePositionCommand;
import org.firstinspires.ftc.teamcode.commands.LiftToScoringPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.commands.auto.BarneyCycleToPoleAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.BarneyCycleToStackAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.BarneyDeliverPreloadAutoCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.BeaconArm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Junctions;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.commands.auto.BarneyAutoTrajectories;

@Autonomous(group = "drive", name = "Barney Blue 1+5")
public class Blue_1_5_Barney extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    final static double fullAutoTime = 30, cycleTime = 5, parkTime = 2;

    boolean finalCycle = false;
    int totalCycles = 0;
    int stackIndex = 4;

    private SampleMecanumDrive_Barney drive;
    private Lift lift;
    private Arm arm;
    private Wrist wrist;
    private Gripper gripper;

    private BarneyCycleToPoleAutoCommand cycleToPoleAutoCommand;
    private BarneyCycleToStackAutoCommand cycleToStackAutoCommand;
    private BarneyDeliverPreloadAutoCommand deliverPreloadAutoCommand;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
        drive = new SampleMecanumDrive_Barney(hardwareMap);
        gripper = new Gripper(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);

        cycleToPoleAutoCommand = new BarneyCycleToPoleAutoCommand(drive, lift, arm, wrist, gripper);
        cycleToStackAutoCommand = new BarneyCycleToStackAutoCommand(drive, lift, arm, wrist, gripper, stackIndex);
        deliverPreloadAutoCommand = new BarneyDeliverPreloadAutoCommand(drive, lift, arm, wrist, gripper, stackIndex);

        drive.setPoseEstimate(BarneyAutoTrajectories.blue_StartPos);
        BarneyAutoTrajectories.generateTrajectories(drive);

        gripper.close();

        while (!isStarted()) {
            telemetry.addLine("Ready for start!");
            telemetry.update();
        }

        schedule(new SequentialCommandGroup(
                deliverPreloadAutoCommand, cycleToPoleAutoCommand,
                new InstantCommand(() -> { stackIndex--; }),
                cycleToStackAutoCommand, cycleToPoleAutoCommand,
                new InstantCommand(() -> { stackIndex--; })
        ));

    }
}



