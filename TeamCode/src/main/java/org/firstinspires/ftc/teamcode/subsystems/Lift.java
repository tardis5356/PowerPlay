package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Random;

@Config
public class Lift extends SubsystemBase {
    private PIDController controller;
    private DcMotorEx mL;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0.2;

    public static int target = 0;
    public static int INTAKE_POSITION = 0, LOW_JUNCTION_POSITION = 10, MEDIUM_JUNCTION_POSITION = 20, HIGH_JUNCTION_POSITION = 30;

    private final double ticks_in_degree = 700 / 180.0;

    public double power = 0;
    public double stickValue = 0;

    public Lift(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        mL = hardwareMap.get(DcMotorEx.class, "mL");

        mL.setDirection(DcMotorEx.Direction.REVERSE);

        mL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        toIntakePosition();
    }

    public void periodic(){
        controller.setPID(p, i, d);
        int liftPos = mL.getCurrentPosition();
        double pid = controller.calculate(liftPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        power = pid + ff + stickValue;

        mL.setPower(power);

//        if(mL.getCurrentPosition() < target - 50) power = 0.5;
//        if(mL.getCurrentPosition() > target + 50) power = -0.5;
    }

    public void setTargetPosition(int targetPos){
        target = targetPos;
    }

    public void toIntakePosition(){
        target = INTAKE_POSITION;
    }

    public void toLowPosition(){
        target = LOW_JUNCTION_POSITION;
    }

    public void toMediumPosition(){
        target = MEDIUM_JUNCTION_POSITION;
    }

    public void toHighPosition(){
        target = HIGH_JUNCTION_POSITION;
    }

    public void manualControl(double stick) {
//        target += stick;
//        mL.setPower(stick);
        stickValue = stick;
    }

    public double getLiftPosition() {
        return mL.getCurrentPosition();
    }

    public double getLiftPower() {
        return power;
    }

    public double getLiftTargetPosition() {
        return target;
    }

}