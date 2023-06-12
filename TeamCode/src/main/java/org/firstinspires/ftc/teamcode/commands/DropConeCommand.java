package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.BatWing;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class DropConeCommand extends SequentialCommandGroup {
    public int initialTarget = Lift.target;
    public DropConeCommand(Gripper gripper, BatWing batwing, Arm arm, Lift lift, Wrist wrist, int newTarget) {
        addCommands(
                new LiftToPositionCommand(lift, newTarget-300, 20),
                new WaitCommand(150),
                new InstantCommand(gripper::open),
                new InstantCommand(arm::toDeliverDropPosition),
                new WaitCommand(500),
new RobotToStateCommand(lift, arm, wrist, gripper, batwing, -10, 0, "intake")
//                new WaitCommand(250),
//                new InstantCommand(batwing::retract)
        );
    }
}