
package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


//@Disabled
@TeleOp(name = "Optimus_TeleOp", group = "Linear Opmode")
public class Optimus_TeleOp extends BaseClass_PP {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    double wristPosition = 0.5;

    @Override
    public void runOpMode() {
        defineComponentsOptimus();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("gripper", sG);
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
//        mL = hardwareMap.get(DcMotor.class, "mL");
//        mA = hardwareMap.get(DcMotor.class, "mA");
//        mR = hardwareMap.get(DcMotor.class, "mR");
//
//        sG = hardwareMap.servo.get("sG");
//        sW = hardwareMap.servo.get("sW");
//
//        mL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        mR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // Most robots need the motor on one side to be reversed to drive forward
//        // Reverse the motor that runs backwards when connected directly to the battery
//        mL.setDirection(DcMotor.Direction.FORWARD);
//        mR.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Gamepad 1 Variables
            double leftY1 = gamepad1.left_stick_y;
            double rightX1 = (gamepad1.right_stick_x) / 2;

            //Gamepad 2 Variables
            double leftY2 = gamepad2.left_stick_y;
            double leftTrigger2 = gamepad2.left_trigger;
            double rightY2 = gamepad2.right_stick_y;
            boolean yButton2 = gamepad2.y;
            double rightTrigger2 = gamepad2.right_trigger;
//            boolean aButton2 = gamepad2.a;
            boolean leftBumper2 = gamepad2.left_bumper;


            boolean rightBumper2 = gamepad2.right_bumper;

            mL.setPower(leftY1 - rightX1);
            mR.setPower(leftY1 + rightX1);
            
//            sG.setPosition(leftY2);

            if (rightTrigger2 != 0) {
                sG.setPosition(0);//0.85
            } else {
                sG.setPosition(1);//0.7
            }
//0.22, 0.35

            //controls wrist, moves in increments
            if (rightBumper2 && wristPosition < 1) {
                wristPosition += .010;
            } else if (leftBumper2 && wristPosition > 0) {
                wristPosition -= .010;
            }

            wristPosition = Range.clip(wristPosition, 0, 1);
            sW.setPosition(wristPosition);

            mA.setPower(-rightY2);





            //if(i eat an orange || i go outside) {
            // i am healthy;
            //}else{
            //i won't have enough vitamins; }










            //0.006
//            // Setup a variable for each drive wheel to save power level for telemetry
//            double leftPower;
//            double rightPower;
//
//            // Choose to drive using either Tank Mode, or POV Mode
//            // Comment out the method that's not used.  The default below is POV.
//
//            // POV Mode uses left stick to go forward, and right stick to turn.
//            // - This uses basic math to combine motions and is easier to drive straight.
//            double drive = -gamepad1.left_stick_y;
//            double turn  =  gamepad1.right_stick_x;
//            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
//            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            //mL.setPower(leftPower);
            //mR.setPower(rightPower);


//            if (rightTrigger2 != 0) {
//                sG.setPosition(0.5);
//            } else {
//                sG.setPosition(0.85);
//            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("wrist position", "" + String.format("%.2f", wristPosition));
            telemetry.addData("gripper position", sG.getPosition());
            telemetry.update();
        }
    }

}