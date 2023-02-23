package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "Primus_New_TeleOp", group = "Linear Opmode")
public class Primus_Empty_Shell_TeleOp extends BaseClass_PP {    // LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    double zeroPosition = 0;    
    boolean encoderReset = false;
    @Override
    public void runOpMode() {

        defineComponentsPrimus();


        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("arm power", mArm.getPower());
            telemetry.addData("arm position", mBR.getCurrentPosition());
            telemetry.addData("zeroPosition", zeroPosition);
            telemetry.update();

///////////////////////////////////////////GAMEPAD VARIABLES//////////////////////////////////////////////////////////////


//////////////////////////////////DRIVETRAIN CONTROLS/////////////////////////////////////////////





/////////////////////////////////////////////IF STATEMENTS//////////////////////////////////////////////////////////////



        }


    }


}

//mArm.setPower(rightY2);

//if (rightTrigger2 != 0) {
//                //sL.setPosition(0.3);
//                sR.setPosition(0.3);
//            } else {
//                sR.setPosition(0.8); //0.3
//            }
//            if (leftTrigger2 != 0) {
//                sL.setPosition(0.7);
//                //sR.setPosition(0.3);
//            } else {
//                sL.setPosition(0.3); //0.3
//            }
