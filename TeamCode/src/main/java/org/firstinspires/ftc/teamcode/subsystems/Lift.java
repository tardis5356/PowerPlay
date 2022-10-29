package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class Lift extends SubsystemBase {
    private PIDController controller;
    private DcMotorEx mL;

    public static double p = 0.8, i = 0, d = 0;
    public static double f = 0.2;

    public static int target = 0;

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

//        toIntakePosition();
    }

    public void periodic(){
        liftPID();
    }

    public void setTargetPosition(int targetPos){
        target = targetPos;
    }

//    public void toIntakePosition(){ target = Junctions.INTAKE_POSITION.position; }
//
//    public void toLowPosition(){
//        target = Junctions.LOW_JUNCTION_POSITION.position;
//    }
//
//    public void toMediumPosition(){
//        target = Junctions.MEDIUM_JUNCTION_POSITION.position;
//    }
//
//    public void toHighPosition(){
//        target = Junctions.HIGH_JUNCTION_POSITION.position;
//    }

    public void manualControl(double stick) {
//        target += stick*5;
//        mL.setPower(stick);
        if(stick < 0) stickValue = stick*0.1;
        else stickValue = stick*0.75;
    }

    public void liftPID() {
        controller.setPID(p, i, d);
        int liftPos = mL.getCurrentPosition();
        double pid = controller.calculate(liftPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        power = pid + ff + stickValue;

        mL.setPower(power);
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

    public boolean atLimit() {
        return getLiftPosition() > 650 || getLiftPosition() < 5;
    }

}