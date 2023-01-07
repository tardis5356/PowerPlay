package org.firstinspires.ftc.teamcode.commands.auto.R2V2;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_HIGH_JUNCTION_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_HIGH_JUNCTION_R2V2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.RobotToStateCommand;
//import org.firstinspires.ftc.teamcode.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Junctions;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.Coffin;


public class R2V2_CycleToPoleAutoCommand extends SequentialCommandGroup {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Gripper gripper;

    public R2V2_CycleToPoleAutoCommand(SampleMecanumDrive_R2V2 drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper, Coffin coffin, boolean isBlue) {
        this.gripper = gripper;

        addCommands(
 //               new WaitCommand(1500),
//                new LiftToPositionCommand(lift, 400, 50),
                new WaitCommand(250),
                new ParallelCommandGroup(
                        new R2V2_FollowTrajectoryCommand(drive, isBlue ? R2V2_AutoTrajectories.blue_StackCloseWaypointToMedPole : R2V2_AutoTrajectories.red_StackWaypointToMainPole),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new RobotToStateCommand(lift, arm, wrist, gripper, coffin, LIFT_HIGH_JUNCTION_R2V2, 0, "delivery"),
                                new WaitCommand(1000),
                                new InstantCommand(gripper::open)
                        )
                )
        );
    }

    @Override
    public boolean isFinished() {
//        return Gripper.getGripperPosition() == Gripper.OPEN_POSITION;
        return super.isFinished();
    }
}
