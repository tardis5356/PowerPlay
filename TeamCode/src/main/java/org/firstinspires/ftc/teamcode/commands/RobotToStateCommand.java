package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_LOW_JUNCTION_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.STACK_POSITIONS_R2V2;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.BatWing;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.teleop.Gen2_TeleOp;

public class RobotToStateCommand extends ParallelCommandGroup {
    public RobotToStateCommand(Lift lift, Arm arm, Wrist wrist, Gripper gripper, BatWing batwing, int height, int stackIndex, String state) {
        switch (state) {
            case "intake":
                addCommands(
                        new LiftToPositionCommand(lift, STACK_POSITIONS_R2V2[stackIndex], 11),
                        new InstantCommand(() -> {
                            arm.toIntakePosition();
                            wrist.toIntakePosition();
                            batwing.storage();
                        })
                );
                break;
            case "delivery":
                addCommands(
                        new SequentialCommandGroup(
                                new LiftToPositionCommand(lift, height, 25),
                                new WaitCommand(1000),//TODO: this doesn't appear to be doing anything
                                new InstantCommand(() -> Gen2_TeleOp.CURRENT_BASE_POWER_MULTIPLIER = Gen2_TeleOp.SLOW_POWER_MULTIPLIER )
                        ),
                        new InstantCommand(() -> {
                            arm.toDeliverPosition();
                            wrist.toDeliverPosition();
                            if (height == LIFT_LOW_JUNCTION_R2V2) {
                                batwing.deployedLowJunction();
                            } else {
                                batwing.deployed();
                            }
                        })
                );
                break;
            case "travel":
                addCommands(
                        new LiftToPositionCommand(lift, height, 25),
                        new InstantCommand(() -> {
                            arm.toTravelPosition();
                            wrist.toTravelPosition();
                            batwing.storage();
                        })
                );
                break;
            case "init":
                addCommands(
                        new LiftToPositionCommand(lift, -5, 25),
                        new InstantCommand(() -> {
                            arm.toInitPosition();
                            wrist.toInitPosition();
                            batwing.storage();
                        })
                );
                break;
            case "autoEnd":
                addCommands(
                        new LiftToPositionCommand(lift, -5, 25),
                        new InstantCommand(() -> {
                            arm.toAutoEndPosition();
                            wrist.toInitPosition();
                            batwing.storage();
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