package org.firstinspires.ftc.teamcode.drive.opmode;

//Imports

public abstract class AutoBase_PP extends BaseClass_PP {
    //Define global variables here

    public void gingerTelemetry() {

        telemetry.addData("gyro", "" + String.format("%.2f deg", gyroZ));
        telemetry.addData("front distance (in)", "" + String.format("%.2f", frontDistance));
        telemetry.addData("front distance filtered (in)", "" + String.format("%.2f", frontDistanceFiltered));
        telemetry.addData("left distance (in)", "" + String.format("%.2f", leftDistance));
        telemetry.addData("left distance filtered (in)", "" + String.format("%.2f", leftDistanceFiltered));
        telemetry.addData("sX position", sX.getPosition());
        telemetry.addData("sYL position", sYL.getPosition());
        telemetry.addData("sYR position", sYR.getPosition());
        telemetry.addData("x pos", pose.x);
        telemetry.addData("y pos", pose.y);
        telemetry.update();
    }
    public void gyroAdjust(double power, double degree) {
        if (gyroZ > degree) {
            rotateClockwise(power);
        } else if (gyroZ < degree) {
            rotateCounterclockwise(power);
        } else {
            return;
        }
    }

    //Sigmoid function for rotating clockwise
    public void rotateSigmoid(double degree) {
        double rotationAggressiveness = 0.08;
        //0.05
        double powerThreshold = 0.15;
        double pMax = 0.5;
        //0.5
        double power = 2 * pMax * ((1 / (1 + Math.pow(Math.E, -(rotationAggressiveness * (gyroZ - degree))))) - 0.5);
        if (power < powerThreshold && power > 0) {
            power = powerThreshold;
        }
        if (power > -powerThreshold && power < 0) {
            power = -powerThreshold;
        }
        //switched from rotateClockwise to rotateCounterclockwise to accommodate turning on testbed - 8/22/21s
        //rotateClockwise(power);
        rotateCounterclockwise(power);
    }

    public void changeStep() {
        runtime.reset();
//        timeAtStop = stopTime.seconds();
        stopDriveTrain();
        isStartRecorded = false;
    }

}