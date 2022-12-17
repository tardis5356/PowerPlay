package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.activeBot;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class Lift extends SubsystemBase {
    private PIDController controller,
            extendController, retractController;
    private DcMotorEx mL_Barney, mBL;
    private CRServo mL_R2V2;

//    public String ACTIVE_BOT = activeBot;

    public static double pE_Barney = BotPositions.LIFT_pE_Barney, pR_Barney = BotPositions.LIFT_pR_Barney, i_Barney = BotPositions.LIFT_i_Barney, d_Barney = BotPositions.LIFT_d_Barney;
    public static double pE_R2V2 = BotPositions.LIFT_p_R2V2, iE_R2V2 = BotPositions.LIFT_i_R2V2, dE_R2V2 = BotPositions.LIFT_d_R2V2;

    public static double f = 0.2; // 0.2 NEEDS TESTING????

    public static int target = 0;

    private final double ticks_in_degree = 700 / 180.0;

    public double power = 0;
    public double stickValue = 0;

    public boolean retract = false;

    public Lift(HardwareMap hardwareMap) {
//        retractController = new PIDController(pR, iR, dR);
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        if(activeBot == 0) {
            controller = new PIDController(pE_Barney, i_Barney, d_Barney);

            mL_Barney = hardwareMap.get(DcMotorEx.class, "mL");

            mL_Barney.setDirection(DcMotorEx.Direction.REVERSE);

            mL_Barney.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mL_Barney.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(activeBot == 1) {
            controller = new PIDController(pE_R2V2, iE_R2V2, dE_R2V2);

            mL_R2V2 = hardwareMap.crservo.get("mL");
//            mL_Barney = hardwareMap.get(DcMotorEx.class, "mL");

            mBL = hardwareMap.get(DcMotorEx.class, "mBL");

//            mBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            mBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//            mL_R2V2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
//        toIntakePosition();
    }

    public void periodic() {
        if(activeBot == 0) liftPID_Barney();
        if(activeBot == 1) liftPID_R2V2();
    }

    public void updatePIDValues() {
        if (retract) controller.setP(pR_Barney);
        else controller.setP(pE_Barney);
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
        if(activeBot == 0) {
            if (stick < 0) stickValue = stick * 0.2;
            else stickValue = stick * 0.75;
        }else{
            stickValue = stick * 1;
        }
    }

    public void liftPID_Barney() {
        int liftPos = mL_Barney.getCurrentPosition();
        double pid = controller.calculate(liftPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        power = pid + ff + stickValue;

        mL_Barney.setPower(power);
    }

    public void liftPID_R2V2() {
        int liftPos = mBL.getCurrentPosition();
        double pid = controller.calculate(liftPos, target);

        power = pid + stickValue;

        mBL.setPower(power);
    }

    public double getLiftPosition() {
        double currentPos = 0;
        if(activeBot == 0) currentPos = mL_Barney.getCurrentPosition();
        if(activeBot == 1) currentPos = mBL.getCurrentPosition();
        return currentPos;
    }

    public double getLiftPower() {
        return power;
    }

    public double getLiftTargetPosition() {
        return target;
    }

    public boolean atLimit() {
        return getLiftPosition() > 650 || getLiftPosition() < 5;
    }

}