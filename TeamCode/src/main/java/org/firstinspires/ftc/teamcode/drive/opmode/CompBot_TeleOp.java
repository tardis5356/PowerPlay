package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "CompBot_TeleOp", group = "Linear Opmode")

public abstract class CompBot_TeleOp extends BaseClass_PP {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        defineComponentsCompBot();
        boolean motorPowerFast = false;
        double powerMultiplier = 0.5;
        boolean previousBState = false;
        double sWPosition = 0;


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            //george = potentiometer.getVoltage();
            telemetry.addData("drivetrain power multiplier", powerMultiplier);
            telemetry.addData("mFL", mFL.getCurrentPosition());
            telemetry.addData("mBL", mBL.getCurrentPosition());
            telemetry.addData("mFR", mFR.getCurrentPosition());
            telemetry.addData("mBR", mBR.getCurrentPosition());
            telemetry.update();
            //Update global sensor values
//            updatePoseStrafe();
            //gyroUpdate();

            //Gamepad 1 Variables
            waitForStart();
            runtime.reset();
            double leftY1 = gamepad1.left_stick_y;
            double rightX1 = (gamepad1.right_stick_x);
            double leftX1 = (gamepad1.left_stick_x);

            //Gamepad 2 Variables
            double leftY2 = (gamepad2.left_stick_y);
            double leftX2 = (gamepad2.left_stick_x);
            double rightX2 = gamepad2.right_stick_x;
            double rightY2 = gamepad2.right_stick_y;
            boolean rightBumper2 = gamepad2.right_bumper;
            boolean leftBumper2 = gamepad2.left_bumper;
            float rightTrigger2 = gamepad2.right_trigger;
            boolean bButton = (gamepad1.b);

            telemetry.addData("leftY2", leftY2);

            drive(leftY1, leftX1, rightX1);


            drive(leftY1, leftX1, rightX1);

            //sets power for lift
            mL.setPower(leftY2);

            //extends tape measure
            sTME.setPower(leftX2);

            //sets power for rotating tape measure
            mTMR.setPower(rightX2);

            //sets power for arm
            sA.setPower(rightY2);

            //controls gripper
            if(rightTrigger2 != 0) {
                sG.setPosition(1);
            }else {
                sG.setPosition(0);
            }

            if (rightBumper2) { //&& sWHPosition < 1)
                sWPosition += .008;
            } else if (leftBumper2) {// && sWHPosition > 0) {
                sWPosition -= .008;
            }
            sW.setPosition(Range.clip(sWPosition, 0, 1));
            sWPosition = sW.getPosition();




            //changes drive speed
            if (bButton != previousBState && bButton) {
                if (motorPowerFast) {
                    motorPowerFast = false;
                    powerMultiplier = 1;
                } else {
                    motorPowerFast = true;
                    powerMultiplier = 0.5;
                }
            }
            previousBState = bButton;


        }
    }
}
