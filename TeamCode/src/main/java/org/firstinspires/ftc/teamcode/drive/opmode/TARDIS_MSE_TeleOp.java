package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TARDIS_MSE_TeleOp", group = "Linear Opmode")
//@Disabled

public class TARDIS_MSE_TeleOp extends BaseClass_PP {    // LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor mFL = null;
//   private DcMotor mFR = null;
// changed motor names in entire program to mFL and mFR from mL and mR

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        defineComponentsSmallz();
        boolean motorPowerFast = false;
        double powerMultiplier = 0.5;
        boolean previousBState = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            //george = potentiometer.getVoltage();
            telemetry.addData("drivetrain power multiplier", powerMultiplier);
            telemetry.addData("mFL", -mFL.getCurrentPosition());
            telemetry.addData("mBL", mBL.getCurrentPosition());
            telemetry.addData("mFR", -mFR.getCurrentPosition());
            telemetry.addData("mBR", mBR.getCurrentPosition());
            telemetry.update();
            //Update global sensor values
//            updatePoseStrafe();
            //gyroUpdate();

            //Gamepad 1 Variables
            waitForStart();
            runtime.reset();
            double leftY1 = gamepad1.left_stick_y * powerMultiplier;
            double rightX1 = -(gamepad1.right_stick_x) / 2.5;
            double leftX1 = -(gamepad1.left_stick_x) * powerMultiplier;

            //Gamepad 2 Variables
            double leftY2 = (gamepad2.left_stick_y);
            boolean bButton = (gamepad1.b);

            telemetry.addData("leftY2", leftY2);

            drive(leftY1, leftX1, rightX1);

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

