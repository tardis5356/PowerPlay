package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.isBarney;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
public class Lift extends SubsystemBase {
    private PIDController controller,
            extendController, retractController;
    private DcMotorEx mL_Barney, mL_R2V2, mL2_R2V2;

    private TouchSensor liftBase;
    public static double pE_Barney = BotPositions.LIFT_pE_Barney, pR_Barney = BotPositions.LIFT_pR_Barney, i_Barney = BotPositions.LIFT_i_Barney, d_Barney = BotPositions.LIFT_d_Barney;
    public static double pE_R2V2 = BotPositions.LIFT_p_R2V2, i_R2V2 = BotPositions.LIFT_i_R2V2, d_R2V2 = BotPositions.LIFT_d_R2V2;

    public static double f_Barney = 0.2;
    public static double f_R2V2 = 0.22;

    public static int target = 0;

    public static int tolerance = 25;

    private final double ticks_in_degree = 700 / 180.0;

    public double power = 0;
    public double stickValue = 0;
    public double stickValue2 = 0;
    public double pid_R2V2 = 0;
    public double ff_R2V2 = 0;
    public boolean manualActive = false;

    public boolean retract = false;

    public int resets = 0;

    public int liftOffset = 0;

    public Lift(HardwareMap hardwareMap) {
//        retractController = new PIDController(pR, iR, dR);
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());


        mL_R2V2 = hardwareMap.get(DcMotorEx.class, "mL");
        mL2_R2V2 = hardwareMap.get(DcMotorEx.class, "mL2");

        if (isBarney) {
            controller = new PIDController(pE_Barney, i_Barney, d_Barney);

            mL_Barney = hardwareMap.get(DcMotorEx.class, "mL");

            mL_Barney.setDirection(DcMotorEx.Direction.REVERSE);

            mL_Barney.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mL_Barney.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            target = 70;
        }
        if (!isBarney) {
            controller = new PIDController(pE_R2V2, i_R2V2, d_R2V2);

            liftBase = hardwareMap.get(TouchSensor.class, "liftBase");


            mL_R2V2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mL_R2V2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mL2_R2V2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            target = -10;
        }
    }

    public void periodic() {
        if (isBarney) liftPID_Barney();
        if (!isBarney) {
            // liftBangBang_R2V2();
//            if (liftOffset == 0 || liftOffset == 1) setLiftOffset();
            liftPID_R2V2();
            //if absolute value of position is greater than tolerance, reset position at base
            if (liftBase.isPressed() && Math.abs(mL_R2V2.getCurrentPosition()) > tolerance) {
                mL_R2V2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mL_R2V2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                resets++;
            }
        }
    }

    public void updatePIDValues() {
        if (isBarney) {
            if (retract) controller.setP(pR_Barney);
            else controller.setP(pE_Barney);
        } else {
//            if (retract) controller.setP(pE_R2V2);
//            else controller.setP(pE_R2V2);
            controller.setPID(pE_R2V2, i_R2V2, d_R2V2);
        }
    }

    public void setTargetPosition(int targetPos) {
        mL_R2V2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mL2_R2V2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double oldTargetPos = target; // get old target before setting new target
        target = targetPos;
        if (targetPos < oldTargetPos) retract = true; // set retraction to true
        else retract = false; // set retraction to false
        updatePIDValues();
        manualActive = false;
    }

    public void setTolerance(int targetTolerance) {
        tolerance = targetTolerance;
    }

    public void setLiftOffset() {
        if (mL_R2V2.getCurrentPosition() > 300 && liftOffset == 0) { // if lift is extended and no offset has been set already
            liftOffset = 1;
        }
        if (liftOffset == 1 && liftBase.isPressed()) { // once lift has extended once and limit is reached, get offset
            liftOffset = mL_R2V2.getCurrentPosition();
        }
    }

    public void manualControl(double stick, double stick2) {
//        controller.setP(0);
        if (isBarney) {
            if (stick < 0) stickValue = stick * 0.2;
            else stickValue = stick * 1;
        } else {
            stickValue = stick * 1;
            if (stickValue2 < 0) stickValue2 = stick2 * 0.2;
            else stickValue2 = stick2 * 0.4;
        }
    }

    public boolean getLiftBase() {
        return liftBase.isPressed();
    }

    public int getLiftBaseResets() {
        return resets;
    }

    public void liftPID_Barney() {
        int liftPos = mL_Barney.getCurrentPosition();
        double pid = controller.calculate(liftPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f_Barney;

//        if (!manualActive) {
        power = pid + ff + stickValue;
//
//            if (Math.abs(stickValue) > 0.05) {
//                manualActive = true;
//            }
//        } else {
//            power = ff + stickValue;
//        }


        mL_Barney.setPower(power);
    }

    public void liftBangBang_R2V2() {
        int liftPos = mL_R2V2.getCurrentPosition();
        int liftTarget = target; // + liftOffset;
        double pid = controller.calculate(liftPos, liftTarget);
//        double ff = -Math.cos(Math.toRadians(target / ticks_in_degree)) * f_R2V2;
        double ff = f_R2V2;

//        power = pid + stickValue;
        pid_R2V2 = pid;
        ff_R2V2 = ff; // - (liftPos / 7000);

        if (!manualActive) {
            if (liftTarget != 10) {
                if (Math.abs(liftTarget - liftPos) > tolerance) {
                    if (liftPos < liftTarget) {
                        power = 1;//1
                    }
                    if (liftPos > liftTarget) {
                        power = -0.4;//-0.3
                    }
                    mL_R2V2.setPower(power);
                    mL2_R2V2.setPower(power);
                } else {
                    mL_R2V2.setPower(ff);
                    mL2_R2V2.setPower(ff);
                }
            }

            if (liftTarget == 10 && liftBase.isPressed()) {
                if (liftBase.isPressed()) {
                    mL_R2V2.setPower(0);
                    mL2_R2V2.setPower(0);
                }
            }
            if (liftTarget == 10 && !liftBase.isPressed()) {
                if (Math.abs(liftTarget - liftPos) > tolerance) {
                    if (liftPos < liftTarget) {
                        power = 1;//1
                    }
                    if (liftPos > liftTarget) {
                        power = -0.4;//-0.3
                    }
                    mL_R2V2.setPower(power);
                    mL2_R2V2.setPower(power);
                } else {
                    mL_R2V2.setPower(ff);
                    mL2_R2V2.setPower(ff);
                }
            }

            if (Math.abs(stickValue + stickValue2) > 0.05) {
                manualActive = true;
            }
        }
        if (manualActive) {
            power = ff + stickValue + stickValue2;
            mL_R2V2.setPower(power);
            mL2_R2V2.setPower(power);
        }

    }

    public void liftPID_R2V2() {
        int liftPos = mL_R2V2.getCurrentPosition();
        int liftTarget = target; // + liftOffset;
        double pid = controller.calculate(liftPos, target);
//        double ff = -Math.cos(Math.toRadians(target / ticks_in_degree)) * f_R2V2;
        double ff = f_R2V2;
        pid_R2V2 = pid;

        if (!manualActive) {

            // lowest position (no motor power) target position is -10
            if (liftTarget != -10) power = pid + ff;

            if (liftTarget == -10 && !liftBase.isPressed()) power = -0.1;
            if (liftTarget == -10 && liftBase.isPressed()) power = 0;

//            if (liftTarget == -10 && liftBase.isPressed()) {
//                mL_R2V2.setPower(0);
//                mL2_R2V2.setPower(0);
//            }
//            if (liftTarget == -10 && !liftBase.isPressed()) {
//                mL_R2V2.setPower(power);
//                mL2_R2V2.setPower(power);
//            }
            mL_R2V2.setPower(power);
            mL2_R2V2.setPower(power);

            if (Math.abs(stickValue + stickValue2) > 0.05) {
                manualActive = true;
            }
        }
        if (manualActive) {
            power = ff + stickValue + stickValue2;
            mL_R2V2.setPower(power);
            mL2_R2V2.setPower(power);
        }

    }

    public double getLiftPosition() {
        double currentPos = 0;
        if (isBarney) currentPos = mL_Barney.getCurrentPosition();
        if (!isBarney) currentPos = target; //mL_R2V2.getCurrentPosition();
        return currentPos;
    }

    public double getLiftPower() {
        return power;
    }

    public double getLiftPID() {
        return pid_R2V2;
    }

    public double getLiftFF() {
        return ff_R2V2;
    }

    public double getLiftTargetPosition() {
        return target;
    }

    public boolean atLimit() {
        return getLiftPosition() > 650 || getLiftPosition() < 5;
    }

}