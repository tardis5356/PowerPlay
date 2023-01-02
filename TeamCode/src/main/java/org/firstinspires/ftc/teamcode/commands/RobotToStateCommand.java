package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Coffin;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class RobotToStateCommand extends ParallelCommandGroup {
    public RobotToStateCommand(Lift lift, Arm arm, Wrist wrist, Gripper gripper, Coffin coffin, int height, int stackIndex, String state) {
        switch(state){
            case "intake":
                addCommands(
                        new LiftToPositionCommand(lift, height + (stackIndex * 48), 15),
                        new InstantCommand(() -> {
                            arm.toIntakePosition();
                            wrist.toIntakePosition();
                            coffin.extend();
                        })
                );
                break;
            case "delivery":
                addCommands(
                        new LiftToPositionCommand(lift, height, 25),
                        new InstantCommand(() -> {
                            arm.toDeliverPosition();
                            wrist.toDeliverPosition();
                            coffin.retract();
                        })
                );
                break;
            case "travel":
                addCommands(
                        new LiftToPositionCommand(lift, height, 25),
                        new InstantCommand(() -> {
                            arm.toTravelPosition();
                            wrist.toTravelPosition();
                            coffin.retract();
                        })
                );
                break;
        }

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