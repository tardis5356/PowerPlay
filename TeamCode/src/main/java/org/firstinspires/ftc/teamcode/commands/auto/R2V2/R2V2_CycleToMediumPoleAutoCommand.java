package org.firstinspires.ftc.teamcode.commands.auto.R2V2;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_HIGH_JUNCTION_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_MEDIUM_JUNCTION_R2V2;

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


public class R2V2_CycleToMediumPoleAutoCommand extends SequentialCommandGroup {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Gripper gripper;

    public R2V2_CycleToMediumPoleAutoCommand(SampleMecanumDrive_R2V2 drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper, Coffin coffin, boolean isBlue) {
        this.gripper = gripper;

        addCommands(
                new ParallelCommandGroup(
                        new R2V2_FollowTrajectoryCommand(drive, isBlue ? R2V2_AutoTrajectories.blue_StackWaypointToMedPole : R2V2_AutoTrajectories.red_StackCloseWaypointToMedPole),  //TODO: spline is slightly wrong
                        new RobotToStateCommand(lift, arm, wrist, gripper, coffin, LIFT_MEDIUM_JUNCTION_R2V2, 0, "delivery")
                ),
                new WaitCommand(1000),
                new InstantCommand(gripper::open)
        );
    }

    @Override
    public boolean isFinished() {
//        return Gripper.getGripperPosition() == Gripper.OPEN_POSITION;
        return super.isFinished();
    }
}
