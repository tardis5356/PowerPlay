package org.firstinspires.ftc.teamcode.subsystems;

public class BotPositions {
    public String activeBot = "Barney";

    public enum Barney {
        LIFT, ARM, WRIST, GRIPPER;

        enum lift_pos {
            p(0.5),
            i(0.76),
            d(0.4);

            public double position;

            lift_pos(double position) {
                this.position = position;
            }
        }

        enum arm_pos {
            INIT(0.5),
            INTAKE(0.76),
            DELIVERY(0.4);

            public double position;

            arm_pos(double position) {
                this.position = position;
            }
        }

        enum wrist_pos {
            INIT(0.5),
            INTAKE(0.76),
            DELIVERY(0.4);

            public double position;

            wrist_pos(double position) {
                this.position = position;
            }
        }

        enum gripper_pos {
            INIT(0.5),
            INTAKE(0.76),
            DELIVERY(0.4);

            public double position;

            gripper_pos(double position) {
                this.position = position;
            }
        }
    }
}
