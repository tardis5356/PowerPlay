package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.isV3PO;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private DcMotorEx mL, mL2;

    private TouchSensor liftBase;
    public static double pE_V3PO = BotPositions.LIFT_p_V3PO, i_V3PO = BotPositions.LIFT_i_V3PO, d_V3PO = BotPositions.LIFT_d_V3PO;
    public static double pE_R2V2 = BotPositions.LIFT_p_R2V2, i_R2V2 = BotPositions.LIFT_i_R2V2, d_R2V2 = BotPositions.LIFT_d_R2V2;

    public static double f_V3PO = 0.1;
    public static double f_R2V2 = 0.22;

    public static int target = 0;

    public static int liftTargetPosButNotTheOtherOne = 0;

    public static int tolerance = 25;

    private final double ticks_in_degree = 700 / 180.0;

    public double power = 0;
    public double stickValue = 0;
    public double stickValue2 = 0;

    public double pid_R2V2 = 0;
    public double ff_R2V2 = 0;

    public double pid_V3PO = 0;
    public double ff_V3PO = 0;

    public boolean manualActive = false;

    public boolean retract = false;

    public int resets = 0;

    public int liftOffset = 0;

    public Lift(HardwareMap hardwareMap) {
//        retractController = new PIDController(pR, iR, dR);
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        if (isV3PO) {
            controller = new PIDController(pE_V3PO, i_V3PO, d_V3PO);
        } else {
            controller = new PIDController(pE_R2V2, i_R2V2, d_R2V2);
        }

        liftBase = hardwareMap.get(TouchSensor.class, "liftBase");

        mL = hardwareMap.get(DcMotorEx.class, "mL");
        mL2 = hardwareMap.get(DcMotorEx.class, "mL2");

        mL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(isV3PO){
            mL.setDirection(DcMotorSimple.Direction.REVERSE);
            mL2.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        target = -10;
    }

    public void periodic() {
        if (isV3PO){
            liftPID_V3PO();
        } else {
            // liftBangBang_R2V2();
//            if (liftOffset == 0 || liftOffset == 1) setLiftOffset();
            liftPID_R2V2();
        }
        //if absolute value of position is greater than tolerance, reset position at base
        if (liftBase.isPressed() && Math.abs(mL.getCurrentPosition()) > tolerance) {
            mL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                resets++;
        }
    }

    public void updatePIDValues() {
        if (isV3PO) {
            controller.setPID(pE_V3PO, i_V3PO, d_V3PO);
        } else {
            controller.setPID(pE_R2V2, i_R2V2, d_R2V2);
        }
    }

    public void setTargetPosition(int targetPos) {
        mL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double oldTargetPos = target; // get old target before setting new target
        target = targetPos;
        if (targetPos < oldTargetPos) retract = true; // set retraction to true
        else retract = false; // set retraction to false
        updatePIDValues();
        manualActive = false;
    }

    public void setTargetButNotTheOtherOne(int targetPos) {
        liftTargetPosButNotTheOtherOne = targetPos;
    }

    public int getTargetButNotTheOtherOne() {
        return liftTargetPosButNotTheOtherOne;
    }

    public void setTolerance(int targetTolerance) {
        tolerance = targetTolerance;
    }

    public void setLiftOffset() {
        if (mL.getCurrentPosition() > 300 && liftOffset == 0) { // if lift is extended and no offset has been set already
            liftOffset = 1;
        }
        if (liftOffset == 1 && liftBase.isPressed()) { // once lift has extended once and limit is reached, get offset
            liftOffset = mL.getCurrentPosition();
        }
    }

    public void manualControl(double stick, double stick2) {
//        controller.setP(0);
        if (isV3PO) {
            stickValue = stick * 1;
            if (stickValue2 < 0) stickValue2 = stick2 * 0.2;
            else stickValue2 = stick2 * 0.4;
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

    public void liftBangBang_R2V2() {
        int liftPos = mL.getCurrentPosition();
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
                    mL.setPower(power);
                    mL2.setPower(power);
                } else {
                    mL.setPower(ff);
                    mL2.setPower(ff);
                }
            }

            if (liftTarget == 10 && liftBase.isPressed()) {
                if (liftBase.isPressed()) {
                    mL.setPower(0);
                    mL2.setPower(0);
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
                    mL.setPower(power);
                    mL2.setPower(power);
                } else {
                    mL.setPower(ff);
                    mL2.setPower(ff);
                }
            }

            if (Math.abs(stickValue + stickValue2) > 0.05) {
                manualActive = true;
            }
        }
        if (manualActive) {
            power = ff + stickValue + stickValue2;
            mL.setPower(power);
            mL2.setPower(power);
        }

    }

    public void liftPID_R2V2() {
        int liftPos = mL.getCurrentPosition();
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
            mL.setPower(power);
            mL2.setPower(power);

            if (Math.abs(stickValue + stickValue2) > 0.05) {
                manualActive = true;
            }
        }
        if (manualActive) {
            power = ff + stickValue + stickValue2;
            mL.setPower(power);
            mL2.setPower(power);
        }

    }

    public void liftPID_V3PO() {
        int liftPos = mL.getCurrentPosition();
        int liftTarget = target; // + liftOffset;
        double pid = controller.calculate(liftPos, target);
//        double ff = -Math.cos(Math.toRadians(target / ticks_in_degree)) * f_V3PO;
        double ff = f_V3PO;
        pid_V3PO = pid;

        if (!manualActive) {

            // lowest position (no motor power) target position is -10
            if (liftTarget != -10) power = pid + ff;

            if (liftTarget == -10 && !liftBase.isPressed()) power = -0.1;
            if (liftTarget == -10 && liftBase.isPressed()) power = 0;

//            if (liftTarget == -10 && liftBase.isPressed()) {
//                mL_V3PO.setPower(0);
//                mL2_V3PO.setPower(0);
//            }
//            if (liftTarget == -10 && !liftBase.isPressed()) {
//                mL_V3PO.setPower(power);
//                mL2_V3PO.setPower(power);
//            }
            mL.setPower(power);
            mL2.setPower(power);

            if (Math.abs(stickValue + stickValue2) > 0.05) {
                manualActive = true;
            }
        }
        if (manualActive) {
            power = ff + stickValue + stickValue2;
            mL.setPower(power);
            mL2.setPower(power);
        }

    }

    public double getLiftPosition() {
        double currentPos = 0;
        if (isV3PO) currentPos = target;
        if (!isV3PO) currentPos = target; //mL_R2V2.getCurrentPosition();
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