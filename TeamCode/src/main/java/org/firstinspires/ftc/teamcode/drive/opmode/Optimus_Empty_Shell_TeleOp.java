
package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


//@Disabled
@TeleOp(name = "Optimus_New_TeleOp", group = "Linear Opmode")
public class Optimus_Empty_Shell_TeleOp extends BaseClass_PP {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    double wristPosition = 0.5;

    @Override
    public void runOpMode() {
        defineComponentsOptimus();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("gripper", sG);
        telemetry.update();


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//////////////////////GAMEPAD VARIABLES//////////////////////////////

    //ex. double leftY1 = gamepad1.left_stick_y
float rightTrigger1 = gamepad1.right_trigger;
float leftTrigger1 = gamepad1.left_trigger;
float rightTrigger2 = gamepad2.right_trigger;
float leftTrigger2 = gamepad2.left_trigger;  /////////////////////////DRIVE FUNCTIONS///////////////////////////////////
 boolean CardieB = gamepad1.a;



//////////////////////IF STATEMENTS////////////////////////////////////////

            //if(i eat an orange || i go outside) {
            // i am healthy;
            //}else{
            //i won't have enough vitamins; }
            if (rightTrigger1 > 0) {
                mL.setPower(1);
            } else if (leftTrigger1>0) {
                              mL.setPower(-1);
            }         else{
                mL.setPower(0);
            }

                if (rightTrigger2 > 0) {
                    mR.setPower(1);
                } else if (leftTrigger2>0) {
                                  mR.setPower(-1);
                }         else{
                    mR.setPower(0);
                }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("wrist position", "" + String.format("%.2f", wristPosition));
            telemetry.addData("gripper position", sG.getPosition());
            telemetry.update();
        }
    }

}

//  mL.setPower(leftY1 - rightX1);
//            mR.setPower(leftY1 + rightX1);
//
//            sG.setPosition(leftY2);

//            if (rightTrigger2 != 0) {
//                sG.setPosition(0);//0.85
//            } else {
//                sG.setPosition(1);//0.7
//            }
//
//
//            //controls wrist, moves in increments
//            if (rightBumper2 && wristPosition < 1) {
//                wristPosition += .010;
//            } else if (leftBumper2 && wristPosition > 0) {
//                wristPosition -= .010;
//            }
//
//            wristPosition = Range.clip(wristPosition, 0, 1);
//            sW.setPosition(wristPosition);
//
//            mA.setPower(-rightY2);
