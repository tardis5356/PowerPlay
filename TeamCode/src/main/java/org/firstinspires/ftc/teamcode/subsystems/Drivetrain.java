package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drivetrain extends SubsystemBase {
    private DcMotorEx mFR, mFL, mBR, mBL;
    private BNO055IMU imu;

    private double offset = 0.0;
    private double powerMultiplier = 1.0;
    private double POWER_MULTIPLIER = 1.0;



    public Drivetrain(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Motors and Other Stuff
        mFR = hardwareMap.get(DcMotorEx .class, "mFR");
        mFL = hardwareMap.get(DcMotorEx.class, "mFL ");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        // Behaviors
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mFL.setDirection(DcMotorSimple.Direction.REVERSE);
        mBL.setDirection(DcMotorSimple.Direction.REVERSE);

        imu.initialize(parameters);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void robotcentric(){
//        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
//        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//        double rx = gamepad1.right_stick_x;
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio, but only when
//        // at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
//
//        mFL.setPower(frontLeftPower);
//        mBL.setPower(backLeftPower);
//        mFR.setPower(frontRightPower);
//        mBR.setPower(backRightPower);
    }

    public void fieldcentric(double ly, double lx, double rx){
        Orientation botOrientationRadians = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        //Add the angle offset to be able to reset the 0 heading, and normalize it back to -pi to pi
        double heading = AngleUnit.normalizeRadians(botOrientationRadians.firstAngle - offset);

//        double ly = -gamepad1.left_stick_y;
//        double lx = gamepad1.left_stick_x;
//        double rx = gamepad1.right_stick_x;

        // Rotate by the heading of the robot
        Vector2d vector = new Vector2d(lx, ly).rotated(-heading);
        lx = vector.getX();
        ly = vector.getY();

        double normalize = Math.max(abs(ly) + abs(lx) + abs(rx), 1.0);

        mFL.setPower((ly + lx + rx) / normalize * powerMultiplier);
        mBL.setPower((ly - lx + rx) / normalize * powerMultiplier);
        mFR.setPower((ly - lx - rx) / normalize * powerMultiplier);
        mBR.setPower((ly + lx - rx) / normalize * powerMultiplier);
    }

    public void antitip(){
        float roll;
        float rollOffset = 0;
        float measuredMaxRoll = 5;

        Orientation botOrientationDegrees = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if(rollOffset == 0) rollOffset = botOrientationDegrees.thirdAngle;

        roll = Math.abs(botOrientationDegrees.thirdAngle) - Math.abs(rollOffset);

        if(roll > measuredMaxRoll) measuredMaxRoll = roll;

        float weightedPowerMultiplier = roll/measuredMaxRoll;

        if(weightedPowerMultiplier > 0.2) {
            powerMultiplier = POWER_MULTIPLIER-weightedPowerMultiplier;
        }
        else {
            powerMultiplier = POWER_MULTIPLIER;
        }

        if (powerMultiplier > 1) powerMultiplier = 1;
    }

}