package org.firstinspires.ftc.teamcode.commands.auto.R2V2;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_INTAKE_AUTO_R2V2;

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


public class R2V2_GrabFromStackCloseWaypointCommand extends SequentialCommandGroup {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Gripper gripper;

    public R2V2_GrabFromStackCloseWaypointCommand(SampleMecanumDrive_R2V2 drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper, Coffin coffin, int stackIndex, boolean isBlue) {
        this.gripper = gripper;

        addCommands(
                new InstantCommand(() -> {
                    coffin.extend();
                }),
                new RobotToStateCommand(lift, arm, wrist, gripper, coffin, LIFT_INTAKE_AUTO_R2V2, stackIndex, "intake"),
                new InstantCommand(gripper::open),
//                new WaitCommand(250),
                new R2V2_FollowTrajectoryCommand(drive, isBlue ? R2V2_AutoTrajectories.blue_StackCloseWaypointToStack : R2V2_AutoTrajectories.red_StackCloseWaypointToStack), //TODO: speed up
                new WaitCommand(500),
                new InstantCommand(() -> {
                    gripper.close();
                }),
//                new WaitCommand(250),
                new ParallelCommandGroup(
                        new R2V2_FollowTrajectoryCommand(drive, isBlue ? R2V2_AutoTrajectories.blue_StackToStackWaypoint : R2V2_AutoTrajectories.red_StackToStackWaypoint),
                        new SequentialCommandGroup(
//                                new WaitCommand(700),
                                new RobotToStateCommand(lift, arm, wrist, gripper, coffin, LIFT_INTAKE_AUTO_R2V2, 0, "travel"))

                )
        );
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
