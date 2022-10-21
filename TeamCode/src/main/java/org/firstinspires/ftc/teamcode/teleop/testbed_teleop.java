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

@TeleOp(name = "testbed teleop")
public class testbed_teleop extends CommandOpMode {

    private DcMotorEx rightFront, leftFront, rightBack, leftBack;
    private BNO055IMU imu;

    private double offset = 0.0;
    private double powerMultiplier = 1.0;

//    double leftY1 = gamepad1.left_stick_y;
//    double rightX1 = -(gamepad1.right_stick_x);
//    double leftX1 = -(gamepad1.left_stick_x);

    //    private GripperSubsystem m_gripper;
//    private GrabStone m_grabCommand;
//    private ReleaseStone m_releaseCommand;
    private Button m_grabButton, m_releaseButton;

    @Override
    public void initialize() {
//        schedule(new BulkCacheCommand(hardwareMap));

        // Object declarations
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
// Motors and Other Stuff
        rightFront = hardwareMap.get(DcMotorEx.class, "mFR");
        leftFront = hardwareMap.get(DcMotorEx.class, "mFL ");
        rightBack = hardwareMap.get(DcMotorEx.class, "mBR");
        leftBack = hardwareMap.get(DcMotorEx.class, "mBL");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
// Behaviors
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu.initialize(parameters);


        telemetry.addLine("Ready to start!");
        telemetry.update();

//        m_gripper = new GripperSubsystem(hardwareMap, "gripper");
//        m_grabCommand = new GrabStone(m_gripper);
//        m_releaseCommand = new ReleaseStone(m_gripper);

//        m_grabButton = (new GamepadButton(m_driverOp, GamepadKeys.Button.A))
//                .whenPressed(m_grabCommand);
//        m_releaseButton = (new GamepadButton(m_driverOp, GamepadKeys.Button.B))
//                .whenPressed(m_releaseCommand);
    }

    @Override
    public void run() {
        super.run();

//        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//
//
//        //Add the angle offset to be able to reset the 0 heading, and normalize it back to -pi to pi
//        double heading = AngleUnit.normalizeRadians(orientation.firstAngle - offset);
//
//        double ly = -gamepad1.left_stick_y;
//        double lx = gamepad1.left_stick_x;
//        double rx = gamepad1.right_stick_x;
//
//        // Rotate by the heading of the robot
//        Vector2d vector = new Vector2d(lx, ly).rotated(-heading);
//        lx = vector.getX();
//        ly = vector.getY();
//
//        double normalize = Math.max(abs(ly) + abs(lx) + abs(rx), 1.0);
//
//        leftFront.setPower((ly + lx + rx) / normalize * powerMultiplier);
//        leftBack.setPower((ly - lx + rx) / normalize * powerMultiplier);
//        rightFront.setPower((ly - lx - rx) / normalize * powerMultiplier);
//        rightBack.setPower((ly + lx - rx) / normalize * powerMultiplier);
    }

}