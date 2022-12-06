package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Lift2 extends SubsystemBase {
    private PIDController controller,
            extendController, retractController;
    private CRServo mL;

    public static double pE = 0.05, pR = 0.01, i = 0, d = 0;
//    public static double pE = 0.05, iE = 0, dE = 0;
//    public static double pR = 0.05, iR = 0, dR = 0;

    public static double f = 0.2; // 0.2 NEEDS TESTING????

    public static int target = 0;

    private final double ticks_in_degree = 700 / 180.0;

    public double power = 0;
    public double stickValue = 0;

    public boolean retract = false;

    public Lift2(HardwareMap hardwareMap) {
        controller = new PIDController(pE, i, d);
//        retractController = new PIDController(pR, iR, dR);
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        mL = hardwareMap.crservo.get("mL");

        //mL.setDirection(DcMotorEx.Direction.REVERSE);

//        mL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        mL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        toIntakePosition();
    }

//    public void periodic() {
//        liftPID();
//    }

    public void updatePIDValues() {
        if (retract) controller.setP(pR);
        else controller.setP(pE);
    }

    public void setTargetPosition(int targetPos) {
        double oldTargetPos = target; // get old target before setting new target
        target = targetPos;
        if (targetPos < oldTargetPos) retract = true; // set retraction to true
        else retract = false; // set retraction to false
        updatePIDValues();
    }

    public void manualControl(double stick) {
        controller.setP(0);
        if (stick < 0) stickValue = stick * 0.2;
        else stickValue = stick * 0.75;
    }

//    public void liftPID() {
//        int liftPos = mL.();
//        double pid = controller.calculate(liftPos, target);
//        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
//
//        power = pid + ff + stickValue;
//
//        mL.setPower(power);
//    }
//
//    public double getLiftPosition() {
//        return mL.getCurrentPosition();
//    }
//
//    public double getLiftPower() {
//        return power;
//    }
//
//    public double getLiftTargetPosition() {
//        return target;
//    }
//
//    public boolean atLimit() {
//        return getLiftPosition() > 650 || getLiftPosition() < 5;
//    }

}