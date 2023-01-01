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
    private DcMotorEx mL_Barney, mBL;
    private CRServo mL_R2V2;

    private TouchSensor liftBase;

    public static double pE_Barney = BotPositions.LIFT_pE_Barney, pR_Barney = BotPositions.LIFT_pR_Barney, i_Barney = BotPositions.LIFT_i_Barney, d_Barney = BotPositions.LIFT_d_Barney;
    public static double pE_R2V2 = BotPositions.LIFT_p_R2V2, i_R2V2 = BotPositions.LIFT_i_R2V2, d_R2V2 = BotPositions.LIFT_d_R2V2;

    public static double f_Barney = 0.2;
    public static double f_R2V2 = -0.2;

    public static int target = 0;

    private final double ticks_in_degree = 700 / 180.0;

    public double power = 0;
    public double stickValue = 0;
    public double pid_R2V2 = 0;
    public double ff_R2V2 = 0;
    public boolean manualActive = false;

    public boolean retract = false;

    public Lift(HardwareMap hardwareMap) {
//        retractController = new PIDController(pR, iR, dR);
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        if(isBarney) {
            controller = new PIDController(pE_Barney, i_Barney, d_Barney);

            mL_Barney = hardwareMap.get(DcMotorEx.class, "mL");

            mL_Barney.setDirection(DcMotorEx.Direction.REVERSE);

            mL_Barney.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mL_Barney.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            target = 70;
        }
        if(!isBarney) {
            controller = new PIDController(pE_R2V2, i_R2V2, d_R2V2);

            liftBase = hardwareMap.get(TouchSensor.class, "liftBase");

            mL_R2V2 = hardwareMap.crservo.get("mL");

            mBL = hardwareMap.get(DcMotorEx.class, "mBL");

            mBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            target = 30;
        }
    }

    public void periodic() {
        if(isBarney) liftPID_Barney();
        if(!isBarney) {
            liftPID_R2V2();
            if(liftBase.isPressed()){
                mBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
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
        manualActive = false;
    }

    public void manualControl(double stick) {
//        controller.setP(0);
        if(isBarney) {
            if (stick < 0) stickValue = stick * 0.2;
            else stickValue = stick * 1;
        }else{
            stickValue = stick * 1;
        }
    }

    public void liftPID_Barney() {
        int liftPos = mL_Barney.getCurrentPosition();
        double pid = controller.calculate(liftPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f_Barney;

        power = pid + ff + stickValue;

        mL_Barney.setPower(power);
    }

    public void liftPID_R2V2() {
        int liftPos = -mBL.getCurrentPosition();
        double pid = -controller.calculate(liftPos, target);
//        double ff = -Math.cos(Math.toRadians(target / ticks_in_degree)) * f_R2V2;
        double ff = f_R2V2;

//        power = pid + stickValue;
        pid_R2V2 = pid;
        ff_R2V2 = ff - (liftPos/7000);

        if(!manualActive) {
            if (Math.abs(target - liftPos) > 25) {
                if (liftPos < target) {
                    power = -1;
                }
                if (liftPos > target) {
                    power = 0.7;
                }
                mL_R2V2.setPower(power + stickValue);
            } else {
                mL_R2V2.setPower(ff + stickValue);
            }

            if(Math.abs(stickValue) > 0.05){
                manualActive = true;
            }
        }
        if(manualActive) {
            power = ff + stickValue;
            mL_R2V2.setPower(power);
        }

    }

    public double getLiftPosition() {
        double currentPos = 0;
        if(isBarney) currentPos = mL_Barney.getCurrentPosition();
        if(!isBarney) currentPos = -mBL.getCurrentPosition();
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