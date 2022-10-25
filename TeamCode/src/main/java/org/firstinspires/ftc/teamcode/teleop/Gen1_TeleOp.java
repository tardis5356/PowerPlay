package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Gripper;

@TeleOp(name = "Gen1 TeleOp")
public class Gen1_TeleOp extends CommandOpMode {
    private DcMotorEx mFR, mFL, mBR, mBL;
    private BNO055IMU imu;

    private double offset = 0.0;
    private double powerMultiplier = 1.0;
    private double POWER_MULTIPLIER = 1.0;

    public Orientation tipOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    public float roll = tipOrientation.secondAngle;

    private Gripper Gripper;
    //    private GrabStone m_grabCommand;
//    private ReleaseStone m_releaseCommand;
    private Button m_grabButton, m_releaseButton;

//    private xControl = new PID();
//    private yControl = new PID();
//    private thetaControl = new PID();

    @Override
    public void initialize() {
//        schedule(new BulkCacheCommand(hardwareMap));

        // Object declarations
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        // Motors and Other Stuff
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mFL = hardwareMap.get(DcMotorEx.class, "mFL ");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Behaviors
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mFL.setDirection(DcMotorSimple.Direction.REVERSE);
        mBL.setDirection(DcMotorSimple.Direction.REVERSE);

        imu.initialize(parameters);


        telemetry.addData("roll", tipOrientation.secondAngle);
        telemetry.addData("powerMultiplier", powerMultiplier);
        telemetry.update();

        Gripper = new Gripper(hardwareMap);
//        m_grabCommand = new GrabStone(m_gripper);
//        m_releaseCommand = new ReleaseStone(m_gripper);

        // Manipulator Triggers
        new Trigger(() -> manipulator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                //If the trigger is pressed and the scoring arm is not in the robot then open the bucket
                .whenActive(() -> {
//                    if (!scoringArm.loading) {
                    Gripper.open();
//                    }
                })
                .whenInactive(() -> {
//                    if (!scoringArm.loading) {
                    Gripper.close();
//                    }
                });

//        m_grabButton = (new GamepadButton(manipulator, GamepadKeys.Button.A))
//                .whenPressed(Gripper.open());
//        m_releaseButton = (new GamepadButton(m_driverOp, GamepadKeys.Button.B))
//                .whenPressed(m_releaseCommand);
    }

    @Override
    public void run() {
        super.run();
//        //FIELDCENTRIC
        Orientation botOrientationRads = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        Orientation botOrientationDegs = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //Add the angle offset to be able to reset the 0 heading, and normalize it back to -pi to pi
        double heading = AngleUnit.normalizeRadians(botOrientationRads.firstAngle - offset);

        double ly = -gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Rotate by the heading of the robot
        Vector2d vector = new Vector2d(lx, ly).rotated(-heading);
        lx = vector.getX();
        ly = vector.getY();

        double normalize = Math.max(abs(ly) + abs(lx) + abs(rx), 1.0);

        mFL.setPower((ly + lx + rx) / normalize * powerMultiplier);
        mBL.setPower((ly - lx + rx) / normalize * powerMultiplier);
        mFR.setPower((ly - lx - rx) / normalize * powerMultiplier);
        mBR.setPower((ly + lx - rx) / normalize * powerMultiplier);


        //ANTI-TIP
//        (m−rmin/rmax−rmin)×(tmax−tmin)+tmin // FORMULA
//        if(rollOffset == 0) rollOffset = botOrientationDegs.thirdAngle;
//
//        roll = Math.abs(botOrientationDegs.thirdAngle) - Math.abs(rollOffset);
//
//        if(roll > measuredMaxRoll) measuredMaxRoll = roll;
//
//        float weightedPowerMultiplier = roll/measuredMaxRoll;
//
////        if(Math.abs(roll) > 5) {
//        powerMultiplier = POWER_MULTIPLIER-weightedPowerMultiplier;
////        }
////        else { powerMultiplier = POWER_MULTIPLIER; }
//
//        if (powerMultiplier > 1) powerMultiplier = 1;
//
//        telemetry.addData("heading", heading);
//        telemetry.addData("roll", roll);
//        telemetry.addData("rollOffset", rollOffset);
//        telemetry.addData("measuredMaxRoll", measuredMaxRoll);
//
//        telemetry.addData("weightedPowerMultiplier", weightedPowerMultiplier);
//        telemetry.addData("powerMultiplier", powerMultiplier);
        telemetry.update();
        /*
        //REV DOCS
        Heading is the measure of rotation along the z-axis.
        If the Hub is laying flat on a table, the z-axis points upwards through the front plate of the Hub.

        Pitch is the measure of rotation along the x-axis.
        The x-axis is the axis that runs from the bottom of the hub, near the servo ports, to the top of the hub ,where the USB ports are.

        Roll is the measure along the y-axis.
        The y-axis is the axis that runs from the sensor ports on the right to the motor ports on the left.

        // TOTES CODE
        public void resetGyro(){
            Orientation i = imu.getAngularOrientation();
            xOffset = i.secondAngle;
            yOffset = i.thirdAngle;
            distanceSensorLocalizer.setGyroOffset(i.firstAngle-Math.toRadians(RobotConstants.getAlliance().selectOf(-90, 90)));
        }
        public void setSafeDrivePower(Pose2d raw){
            Orientation i = imu.getAngularOrientation();
            float x = 0, y = 0, adjX = xOffset-i.secondAngle, adjY = i.thirdAngle-yOffset;
            if(Math.abs(adjY) > TIP_TOLERANCE) y = adjY;
            if(Math.abs(adjX) > TIP_TOLERANCE) x = adjX;
    //        setWeightedDrivePower(raw.plus(new Pose2d(x>0 ? Math.max(x-TIP_TOLERANCE, 0) : Math.min(x+TIP_TOLERANCE, 0), y>0 ? Math.max(y-TIP_TOLERANCE, 0) : Math.min(y+TIP_TOLERANCE, 0), 0)));
            setWeightedDrivePower(raw.plus(new Pose2d(Range.clip(x*2, -TIP_AUTHORITY, TIP_AUTHORITY), Range.clip(y*2, -TIP_AUTHORITY, TIP_AUTHORITY), 0)));
        }
         */
    }

}