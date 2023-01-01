package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_FULL_RETRACTION_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_HIGH_JUNCTION_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_INTAKE_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_LOW_JUNCTION_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_MEDIUM_JUNCTION_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_TRAVEL_R2V2;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.commands.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.BeaconArm;
import org.firstinspires.ftc.teamcode.subsystems.Coffin;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TapeMeasure;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@TeleOp(name = "Gen2_TeleOp")
public class Gen2_TeleOp extends CommandOpMode {
    private DcMotorEx mFR, mFL, mBR, mBL;
    private BNO055IMU imu;

    private double offset = 0.0;
    private double powerMultiplier = 1.0;
    private double SLOW_POWER_MULTIPLIER = 0.5;
    private double FAST_POWER_MULTIPLIER = 1.0;
    private boolean manualModeOn = false;

    private Drivetrain Drivetrain;
    private Lift lift;
    private Arm arm;
    private Wrist wrist;
    private Gripper gripper;
    private BeaconArm beaconArm;
    private TapeMeasure tapeMeasure;
    private Coffin coffin;

    private RobotToStateCommand liftRetractCommand, liftToTravelPositionCommand, liftToLowJunctionCommand, liftToMediumJunctionCommand, liftToHighJunctionCommand;
    private RobotToStateCommand liftToIntakeCommand;
    private ManualLiftCommand manualLiftCommand;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Object declarations
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        // Motors and Other Stuff
        //defineComponents
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

        telemetry.addData("Ready to start!", getRuntime());
        telemetry.update();

        Drivetrain = new Drivetrain(hardwareMap);
//        defining subsystems
        gripper = new Gripper(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        beaconArm = new BeaconArm(hardwareMap);
        tapeMeasure = new TapeMeasure(hardwareMap);
        coffin = new Coffin(hardwareMap);

        liftToIntakeCommand = new RobotToStateCommand(lift, arm, wrist, gripper, coffin, LIFT_INTAKE_R2V2, 0, "intake");
        liftRetractCommand = new RobotToStateCommand(lift, arm, wrist, gripper, coffin, LIFT_FULL_RETRACTION_R2V2, 0, "delivery");
        liftToTravelPositionCommand = new RobotToStateCommand(lift, arm, wrist, gripper, coffin, LIFT_TRAVEL_R2V2, 0, "travel");
        liftToLowJunctionCommand = new RobotToStateCommand(lift, arm, wrist, gripper, coffin, LIFT_LOW_JUNCTION_R2V2, 0, "delivery");
        liftToMediumJunctionCommand = new RobotToStateCommand(lift, arm, wrist, gripper, coffin, LIFT_MEDIUM_JUNCTION_R2V2, 0, "delivery");
        liftToHighJunctionCommand = new RobotToStateCommand(lift, arm, wrist, gripper, coffin, LIFT_HIGH_JUNCTION_R2V2, 0, "delivery");
        manualLiftCommand = new ManualLiftCommand(lift, manipulator.getLeftY());

        // driver triggers
        //driver = gamepad 1

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(() -> {
                    powerMultiplier = SLOW_POWER_MULTIPLIER;
                });
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(() -> {
                    powerMultiplier = FAST_POWER_MULTIPLIER;
                });

        //teleOp manual mode
        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.DPAD_UP))
                .toggleWhenActive(() -> {
                    manualModeOn = true;
                }, () -> {
                    manualModeOn = false;
                });

        // manipulator triggers
        //manipulator = gamepad 2
        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.A)) // extend to ground junction and slow drive base on A button
                .whenActive(liftToTravelPositionCommand)
                .whenActive(() -> powerMultiplier = SLOW_POWER_MULTIPLIER)
                .cancelWhenActive(liftToHighJunctionCommand)
                .cancelWhenActive(liftToMediumJunctionCommand)
                .cancelWhenActive(liftToLowJunctionCommand)
                .cancelWhenActive(liftToIntakeCommand)
                .cancelWhenActive(manualLiftCommand)
                .cancelWhenActive(liftRetractCommand);
        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.X)) // extend to low junction and slow drive base on B button
                .whenActive(liftToLowJunctionCommand)
                .whenActive(() -> powerMultiplier = SLOW_POWER_MULTIPLIER)
                .cancelWhenActive(liftToHighJunctionCommand)
                .cancelWhenActive(liftToMediumJunctionCommand)
                .cancelWhenActive(liftToTravelPositionCommand)
                .cancelWhenActive(liftToIntakeCommand)
                .cancelWhenActive(manualLiftCommand)
                .cancelWhenActive(liftRetractCommand);
        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.Y)) // extend to medium junction and slow drive base on Y button
                .whenActive(liftToMediumJunctionCommand)
                .whenActive(() -> powerMultiplier = SLOW_POWER_MULTIPLIER)
                .cancelWhenActive(liftToHighJunctionCommand)
                .cancelWhenActive(liftToLowJunctionCommand)
                .cancelWhenActive(liftToTravelPositionCommand)
                .cancelWhenActive(liftToIntakeCommand)
                .cancelWhenActive(manualLiftCommand)
                .cancelWhenActive(liftRetractCommand);
        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.B)) // extend to high junction and slow drive base on X button
                .whenActive(liftToHighJunctionCommand)
                .whenActive(() -> powerMultiplier = SLOW_POWER_MULTIPLIER)
                .cancelWhenActive(liftToMediumJunctionCommand)
                .cancelWhenActive(liftToLowJunctionCommand)
                .cancelWhenActive(liftToTravelPositionCommand)
                .cancelWhenActive(liftToIntakeCommand)
                .cancelWhenActive(manualLiftCommand)
                .cancelWhenActive(liftRetractCommand);

//        new Trigger(() -> manipulator.getLeftY() > 0.2) // override all other commands and give manual control of lift
//                .whenActive(manualLiftCommand)
//                .whenActive(() -> powerMultiplier = SLOW_POWER_MULTIPLIER)
//                .cancelWhenActive(liftToHighJunctionCommand)
//                .cancelWhenActive(liftToMediumJunctionCommand)
//                .cancelWhenActive(liftToLowJunctionCommand)
//                .cancelWhenActive(liftToGroundJunctionCommand)
//                .cancelWhenActive(liftToIntakeCommand)
//                .cancelWhenActive(liftRetractCommand);

        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.DPAD_DOWN)) // retract to intake and speed up drive base on DOWN button
                .whenActive(liftToIntakeCommand)
                .whenActive(() -> powerMultiplier = FAST_POWER_MULTIPLIER)
                .cancelWhenActive(liftToHighJunctionCommand)
                .cancelWhenActive(liftToMediumJunctionCommand)
                .cancelWhenActive(liftToLowJunctionCommand)
                .cancelWhenActive(liftToTravelPositionCommand)
                .cancelWhenActive(liftRetractCommand)
                .cancelWhenActive(manualLiftCommand);

        new Trigger(() -> manipulator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) // closes gripper on left trigger
                .whenActive(() -> {
                    if (!manualModeOn) {
                        gripper.close();
                    } else {
                        arm.increasePosition();
                    }
                });
        new Trigger(() -> manipulator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) // opens gripper on right trigger
                .whenActive(() -> {
                    if (!manualModeOn) {
                        gripper.open();
                    } else {
                        arm.decreasePosition();
                    }
                });


        new Trigger(() -> driver.getButton(GamepadKeys.Button.LEFT_BUMPER)) // move beacon arm to loading position
                .whenActive(() -> beaconArm.toLoadingPosition());
        new Trigger(() -> driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)) // move beacon arm to scoring position
                .whenActive(() -> beaconArm.toDeliveryPosition());
        new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_UP)) // move beacon arm to storage position
                .whenActive(() -> beaconArm.toStoragePosition());

        new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_LEFT))
                .whileActiveContinuous(() -> tapeMeasure.retract());
        new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_RIGHT))
                .whileActiveContinuous(() -> tapeMeasure.extend());
        new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_DOWN))
                .whileActiveContinuous(() -> tapeMeasure.stop());


        new Trigger(() -> driver.getButton(GamepadKeys.Button.A))
                .whenActive(() -> {
                    offset = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
                });
    }

    @Override
    public void run() {
        super.run();
        //FIELDCENTRIC
        Orientation botOrientationRadians = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        //Add the angle offset to be able to reset the 0 heading, and normalize it back to -pi to pi
        double heading = AngleUnit.normalizeRadians(botOrientationRadians.firstAngle - offset);

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

        if (manualModeOn) {

            lift.manualControl(gamepad2.left_stick_y);

            //controls gripper
            if (gamepad2.dpad_right) {
                gripper.increasePosition();

            }
            if (gamepad2.dpad_left) {
                gripper.decreasePosition();
            }

            //controls wrist
            if (gamepad2.right_bumper) {
                wrist.increasePosition();

            }
            if (gamepad2.left_bumper) {
                wrist.decreasePosition();
            }

            //controls arm
            if (gamepad2.right_trigger == 1) {
                arm.increasePosition();

            }
            if (gamepad2.left_trigger == 1) {
                arm.decreasePosition();
            }

        }
//        if(gamepad2.right_bumper){
//            gripper.close();
//        }
//        if(gamepad2.left_bumper){
//            gripper.open();
//        }

//        gripper.open();


        lift.manualControl(gamepad2.left_stick_y);

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
        telemetry.addData("lift pos", lift.getLiftPosition());
        telemetry.addData("lift power", lift.getLiftPower());
        telemetry.addData("lift target", lift.target);
        telemetry.addData("lift pid", lift.getLiftPID());
        telemetry.addData("lift ff", lift.getLiftFF());


//        telemetry.addData("arm pos", arm.getArmPosition());
//        telemetry.addData("gripper pos", gripper.getGripperPosition());
//        telemetry.addData("beacon pos", beaconArm.getBeaconArmPosition());
//        telemetry.addData("wrist pos", String.format("%.2f", wrist.getWristPosition()));

//        telemetry.addData("manual mode is", manualModeOn);
//
//        telemetry.addLine("odometers");
//        telemetry.addData("right: ", mFR.getCurrentPosition());
//        telemetry.addData("  back: ", mBR.getCurrentPosition());
//        telemetry.addData("  left: ", mFL.getCurrentPosition());

        telemetry.update();
//    }
//
    }
}
