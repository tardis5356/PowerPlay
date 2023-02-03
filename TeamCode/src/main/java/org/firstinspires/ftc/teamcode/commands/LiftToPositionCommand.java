package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LiftToPositionCommand extends CommandBase {
    private Lift lift;

    int targetPosition;
    int tolerance;

    public LiftToPositionCommand(Lift lift, int targetPosition, int tolerance) {
        this.lift = lift;
        this.targetPosition = targetPosition;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() { // runs once
//        lift.setTargetPosition(targetPosition);
    }

    @Override
    public void execute() { // runs continuously
        lift.setTolerance(tolerance);
        lift.setTargetPosition(targetPosition);
    }

    @Override
    public boolean isFinished() { // returns true when finished
        return Math.abs(lift.getLiftPosition() - targetPosition) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
//        lift.stop();
    }

}
