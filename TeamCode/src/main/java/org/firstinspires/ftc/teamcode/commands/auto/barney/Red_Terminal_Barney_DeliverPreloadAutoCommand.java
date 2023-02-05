package org.firstinspires.ftc.teamcode.commands.auto.barney;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_HIGH_JUNCTION_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_INTAKE_Barney;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.BatWing;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class Red_Terminal_Barney_DeliverPreloadAutoCommand extends SequentialCommandGroup {
    private Gripper gripper;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Red_Terminal_Barney_DeliverPreloadAutoCommand(SampleMecanumDrive_Barney drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper, BatWing coffin, int stackIndex, boolean isBlue) {
        this.gripper = gripper;

        addCommands(
                new ParallelCommandGroup(
                        new Barney_FollowTrajectoryCommand(drive, Barney_AutoTrajectories.red_StartToPreloadPole),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new RobotToStateCommand(lift, arm, wrist, gripper, coffin, LIFT_HIGH_JUNCTION_Barney, 0, "delivery"),
                                new WaitCommand(500)
                        )
                ),
                new WaitCommand(1000),
                new InstantCommand(gripper::open),
                new ParallelCommandGroup(
                        new Barney_FollowTrajectoryCommand(drive, Barney_AutoTrajectories.blue_PreloadPoleToStackWaypoint),
                        new SequentialCommandGroup(
                                new WaitCommand(250),
                                new RobotToStateCommand(lift, arm, wrist, gripper, coffin, LIFT_INTAKE_Barney, stackIndex, "intake")
                        ),
                        new InstantCommand(gripper::open)
                )
        );
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
