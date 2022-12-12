package org.firstinspires.ftc.teamcode.subsystems;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.config.Config;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.util.RegressionUtil;

import java.util.Arrays;

@Config
public class BotPositions {
    //    public String activeBot = "Barney";
    public static int activeBot = 1;

    public static double LIFT_FULL_RETRACTION, LIFT_INTAKE, LIFT_GROUND_JUNCTION, LIFT_LOW_JUNCTION, LIFT_MEDIUM_JUNCTION, LIFT_HIGH_JUNCTION;

    public static double LIFT_pE, LIFT_pR, LIFT_iE, LIFT_dE;

    public static double ARM_INIT, ARM_INTAKE, ARM_DELIVERY;

    public static double WRIST_INIT, WRIST_INTAKE, WRIST_DELIVERY;

    public static double GRIPPER_OPEN, GRIPPER_CLOSED;

    public static double BEACON_ARM_LOAD, BEACON_ARM_STORAGE, BEACON_ARM_DELIVERY;

    public enum Barney {
        LIFT, ARM, WRIST, GRIPPER, BEACON_ARM;

        enum lift {
            pE(0.07),
            pR(0.0001),
            i(0),
            d(0),

            FULL_RETRACTION(0), // 0
            INTAKE(30), // 100
            GROUND_JUNCTION(100),
            LOW_JUNCTION(350), // 200
            MEDIUM_JUNCTION(670), // 450
            HIGH_JUNCTION(800);// 600

            public double position;

            lift(double position) {
                this.position = position;
            }
        }

        enum arm {
            INIT(0.5),
            INTAKE(0.76),
            DELIVERY(0.4);

            public double position;

            arm(double position) {
                this.position = position;
            }
        }

        enum wrist {
            INIT(0.8),
            INTAKE(0.3),
            DELIVERY(0.9);

            public double position;

            wrist(double position) {
                this.position = position;
            }
        }

        enum gripper {
            INIT(0.3),
            CLOSED(0.3),
            OPEN(0.52);

            public double position;

            gripper(double position) {
                this.position = position;
            }
        }

        enum beacon_arm {
            LOAD(0.99),
            DELIVERY(0.43),
            STORAGE(0.5);

            public double position;

            beacon_arm(double position) {
                this.position = position;
            }
        }
    }

    public enum R2V2 {
        LIFT, ARM, WRIST, GRIPPER;

        enum lift {
            p(0.01),
            i(0),
            d(0),

            FULL_RETRACTION(0), // 0
            INTAKE(30), // 100
            GROUND_JUNCTION(100),
            LOW_JUNCTION(350), // 200
            MEDIUM_JUNCTION(670), // 450
            HIGH_JUNCTION(800);// 600

            public double position;

            lift(double position) {
                this.position = position;
            }
        }

        enum arm {
            INIT(0.5),
            INTAKE(0.76),
            DELIVERY(0.4);

            public double position;

            arm(double position) {
                this.position = position;
            }
        }

        enum wrist {
            INIT(0.8),
            INTAKE(0.3),
            DELIVERY(0.9);

            public double position;

            wrist(double position) {
                this.position = position;
            }
        }

        enum gripper {
            INIT(0.3),
            CLOSED(0.3),
            OPEN(0.52);

            public double position;

            gripper(double position) {
                this.position = position;
            }
        }

        enum beacon_arm {
            LOAD(0.99),
            DELIVERY(0.43),
            STORAGE(0.5);

            public double position;

            beacon_arm(double position) {
                this.position = position;
            }
        }
    }

    public static void setActiveBot() {
        if (activeBot == 0) {
            LIFT_FULL_RETRACTION = Barney.lift.FULL_RETRACTION.position;
            LIFT_INTAKE = Barney.lift.INTAKE.position;
            LIFT_GROUND_JUNCTION = Barney.lift.GROUND_JUNCTION.position;
            LIFT_LOW_JUNCTION = Barney.lift.LOW_JUNCTION.position;
            LIFT_MEDIUM_JUNCTION = Barney.lift.MEDIUM_JUNCTION.position;
            LIFT_HIGH_JUNCTION = Barney.lift.HIGH_JUNCTION.position;

            LIFT_pE = Barney.lift.pE.position;
            LIFT_pR = Barney.lift.pR.position;
            LIFT_iE = Barney.lift.i.position;
            LIFT_dE = Barney.lift.d.position;

            ARM_INIT = Barney.arm.INIT.position;
            ARM_INTAKE = Barney.arm.INTAKE.position;
            ARM_DELIVERY = Barney.arm.DELIVERY.position;

            WRIST_INIT = Barney.wrist.INIT.position;
            WRIST_INTAKE = Barney.wrist.INTAKE.position;
            WRIST_DELIVERY = Barney.wrist.DELIVERY.position;

            GRIPPER_OPEN = Barney.gripper.OPEN.position;
            GRIPPER_CLOSED = Barney.gripper.CLOSED.position;

            BEACON_ARM_LOAD = Barney.beacon_arm.LOAD.position;
            BEACON_ARM_STORAGE = Barney.beacon_arm.STORAGE.position;
            BEACON_ARM_DELIVERY = Barney.beacon_arm.DELIVERY.position;
        }
        if (activeBot == 1) {
            LIFT_FULL_RETRACTION = R2V2.lift.FULL_RETRACTION.position;
            LIFT_INTAKE = R2V2.lift.INTAKE.position;
            LIFT_GROUND_JUNCTION = R2V2.lift.GROUND_JUNCTION.position;
            LIFT_LOW_JUNCTION = R2V2.lift.LOW_JUNCTION.position;
            LIFT_MEDIUM_JUNCTION = R2V2.lift.MEDIUM_JUNCTION.position;
            LIFT_HIGH_JUNCTION = R2V2.lift.HIGH_JUNCTION.position;

            LIFT_pE = R2V2.lift.p.position;
            LIFT_iE = R2V2.lift.i.position;
            LIFT_dE = R2V2.lift.d.position;

            ARM_INIT = R2V2.arm.INIT.position;
            ARM_INTAKE = R2V2.arm.INTAKE.position;
            ARM_DELIVERY = R2V2.arm.DELIVERY.position;

            WRIST_INIT = R2V2.wrist.INIT.position;
            WRIST_INTAKE = R2V2.wrist.INTAKE.position;
            WRIST_DELIVERY = R2V2.wrist.DELIVERY.position;

            GRIPPER_OPEN = R2V2.gripper.OPEN.position;
            GRIPPER_CLOSED = R2V2.gripper.CLOSED.position;

            BEACON_ARM_LOAD = R2V2.beacon_arm.LOAD.position;
            BEACON_ARM_STORAGE = R2V2.beacon_arm.STORAGE.position;
            BEACON_ARM_DELIVERY = R2V2.beacon_arm.DELIVERY.position;

        }
    }
}
