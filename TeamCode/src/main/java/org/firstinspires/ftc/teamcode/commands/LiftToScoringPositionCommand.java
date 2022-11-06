package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

import org.firstinspires.ftc.teamcode.subsystems.Junctions;

public class LiftToScoringPositionCommand extends ParallelCommandGroup {
    public LiftToScoringPositionCommand(Lift lift, Arm arm, Gripper gripper, Wrist wrist, Junctions junction) {
        addCommands(
                new LiftToPositionCommand(lift, junction.position, 5),
                new InstantCommand(() -> {
                    arm.toDeliverPosition();
                    wrist.toDeliverPosition();
                })
        );
    }
}

//public class LiftToScoringPositionCommand extends SequentialCommandGroup {
//    public LiftToScoringPositionCommand(Lift lift, Arm arm, Gripper gripper, Wrist wrist, Junctions junction) {
//        new ParallelCommandGroup(
//                new LiftToPositionCommand(lift, junction.position, 5),
//                new InstantCommand(() -> {
//                    arm.toDeliverPosition();
//                    new SequentialCommandGroup(
//                            new WaitCommand(200),
//                            new InstantCommand(() -> {
//                                wrist.toDeliverPosition();
//                            })
//                    );
//                })
//        );
//    }
//}