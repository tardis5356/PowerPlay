package org.firstinspires.ftc.teamcode.commands.auto.R2V2;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_HIGH_JUNCTION_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_INTAKE_R2V2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Coffin;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;


public class R2V2_DeliverPreloadCloseWaypointAutoCommand extends SequentialCommandGroup {
    private Gripper gripper;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public R2V2_DeliverPreloadCloseWaypointAutoCommand(SampleMecanumDrive_R2V2 drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper, Coffin coffin, int stackIndex, boolean isBlue) {
        this.gripper = gripper;

        addCommands(
//                new ParallelCommandGroup(
////                        new R2V2_FollowTrajectoryCommand(drive, isBlue ? R2V2_AutoTrajectories.blue_StartToPreloadPole : R2V2_AutoTrajectories.red_StartToPreloadPole),
//                        new SequentialCommandGroup(
                new WaitCommand(1000), // TODO: the lift is lifting as soon as program starts sometimes?
                new RobotToStateCommand(lift, arm, wrist, gripper, coffin, LIFT_HIGH_JUNCTION_R2V2, 0, "delivery"),
                new WaitCommand(500)
//                        )
//                )//,
//                new WaitCommand(250),
//                new InstantCommand(gripper::open),
//                new ParallelCommandGroup(
//                        new R2V2_FollowTrajectoryCommand(drive, isBlue ? R2V2_AutoTrajectories.blue_PreloadPoleToStackCloseWaypoint : R2V2_AutoTrajectories.red_PreloadPoleToStackCloseWaypoint),
//                        new SequentialCommandGroup(
//                                new WaitCommand(500),
//                                new RobotToStateCommand(lift, arm, wrist, gripper, coffin, LIFT_INTAKE_R2V2, 0, "intake")
//                        )
//                        //new InstantCommand(gripper::open)
//                )
        );
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
