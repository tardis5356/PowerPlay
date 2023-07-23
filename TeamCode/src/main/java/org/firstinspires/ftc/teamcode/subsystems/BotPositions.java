package org.firstinspires.ftc.teamcode.subsystems;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.config.Config;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.util.RegressionUtil;

import java.util.Arrays;

@Config
public class BotPositions {
    public static boolean isV3PO = true;
    //true = V3PO
    //false = R2V2/V3PO

    public static int LIFT_INTAKE_AUTO_V3PO = 25, LIFT_INTAKE_V3PO = -10, LIFT_TRAVEL_V3PO = -10, LIFT_LOW_JUNCTION_V3PO = 200, LIFT_MEDIUM_JUNCTION_V3PO = 640, LIFT_HIGH_JUNCTION_V3PO = 1050, AUTO_LIFT_HIGH_JUNCTION_V3PO = 1100;//1175

    public static int[] STACK_POSITIONS_V3PO = {-10, 45, 100, 170, 210, 250};//0 = bottom cone, 4 = top cone MTI SUBMISSION

    public static double LIFT_p_V3PO = 0.01, LIFT_i_V3PO = 0, LIFT_d_V3PO = 0.0001;

    public static double ARM_INIT_V3PO = 0.5, ARM_INTAKE_V3PO = 0.08, ARM_AUTO_INTAKE_WAYPOINT_V3PO = 0.60, ARM_DELIVERY_V3PO = 0.78, ARM_DELIVERY_DROP_V3PO = 0.9, ARM_TRAVEL_V3PO = 0.53, ARM_STORAGE_V3PO = 0.5, ARM_AUTO_END_V3PO = 0.48; //old deliver 0.365

    public static double WRIST_INIT_V3PO = 0.78, WRIST_INTAKE_V3PO = 1, WRIST_AUTO_INTAKE_WAYPOINT_V3PO = 0.05, WRIST_DELIVERY_V3PO = 0.3, WRIST_TRAVEL_V3PO = 0.3;

    public static double GRIPPER_OPEN_V3PO = 0.6, GRIPPER_CLOSED_V3PO = 0.35, GRIPPER_OPEN_AUTO_V3PO = 0.6;

    public static double BEACON_ARM_LOAD_V3PO = 1, BEACON_ARM_STORAGE_V3PO = 0.6, BEACON_ARM_DELIVERY_V3PO = 0.8;

    public static double BATWING_DEPLOYED_V3PO = 0.58, BATWING_RETRACTED_V3PO = 0.8, BATWING_STORAGE_V3PO = 0.2, BATWING_DEPLOYED_LOW_JUNCTION_V3PO = 0.62;


    public static int LIFT_INTAKE_AUTO_R2V2 = 25, LIFT_INTAKE_R2V2 = -10, LIFT_TRAVEL_R2V2 = -10, LIFT_LOW_JUNCTION_R2V2 = 350, LIFT_MEDIUM_JUNCTION_R2V2 = 770, LIFT_HIGH_JUNCTION_R2V2 = 1150, AUTO_LIFT_HIGH_JUNCTION_R2V2 = 1100;//1175

//    public static int[] STACK_POSITIONS_R2V2 = {-10, 65, 85, 140, 180, 210};//0 = bottom cone, 4 = top cone
//public static int[] STACK_POSITIONS_R2V2 = {-10, 0, 65, 130, 170, 200};//0 = bottom cone, 4 = top cone EXC CHAMPS
//    public static int[] STACK_POSITIONS_R2V2 = {-10, 45, 100, 170, 210, 250};//0 = bottom cone, 4 = top cone MTI SUBMISSION
    public static int[] STACK_POSITIONS_R2V2 = {-10, 45, 100, 170, 210, 250};//0 = bottom cone, 4 = top cone MTI SUBMISSION

    public static double LIFT_p_R2V2 = 0.01, LIFT_i_R2V2 = 0, LIFT_d_R2V2 = 0.0001;

    public static double ARM_INIT_R2V2 = 0.565, ARM_INTAKE_R2V2 = 0.65, ARM_AUTO_INTAKE_WAYPOINT_R2V2 = 0.60, ARM_DELIVERY_R2V2 = 0.39, ARM_TRAVEL_R2V2 = 0.53, ARM_STORAGE_R2V2 = 0.5, ARM_AUTO_END_R2V2 = 0.48; //old deliver 0.365

    public static double WRIST_INIT_R2V2 = 0.78, WRIST_INTAKE_R2V2 = 0.35, WRIST_AUTO_INTAKE_WAYPOINT_R2V2 = 0.05, WRIST_DELIVERY_R2V2 = 0.95, WRIST_TRAVEL_R2V2 = 0.3;

    public static double GRIPPER_OPEN_R2V2 = 0.5, GRIPPER_CLOSED_R2V2 = 0.16, GRIPPER_OPEN_AUTO_R2V2 = 0.6;

    public static double BEACON_ARM_LOAD_R2V2 = 1, BEACON_ARM_STORAGE_R2V2 = 0.6, BEACON_ARM_DELIVERY_R2V2 = 0.8;

    public static double BATWING_DEPLOYED_R2V2 = 0.58, BATWING_RETRACTED_R2V2 = 0.8, BATWING_STORAGE_R2V2 = 0.2, BATWING_DEPLOYED_LOW_JUNCTION_R2V2 = 0.62;

    //V3PO?
    public static int[] STACK_POSITIONS_V3PO = {-10, 45, 100, 170, 210, 250};//0 = bottom cone, 4 = top cone MTI SUBMISSION

    public static double LIFT_p_V3PO = 0.01, LIFT_i_V3PO = 0, LIFT_d_V3PO = 0.0001;

    public static double ARM_INIT_V3PO = 0.565, ARM_INTAKE_V3PO = 0.65, ARM_AUTO_INTAKE_WAYPOINT_V3PO = 0.60, ARM_DELIVERY_V3PO = 0.39, ARM_TRAVEL_V3PO = 0.53, ARM_STORAGE_V3PO = 0.5, ARM_AUTO_END_V3PO = 0.48; //old deliver 0.365

    public static double WRIST_INIT_V3PO = 0.78, WRIST_INTAKE_V3PO = 0.35, WRIST_AUTO_INTAKE_WAYPOINT_V3PO = 0.05, WRIST_DELIVERY_V3PO = 0.95, WRIST_TRAVEL_V3PO = 0.3;

    public static double GRIPPER_OPEN_V3PO = 0.5, GRIPPER_CLOSED_V3PO = 0.16, GRIPPER_OPEN_AUTO_V3PO = 0.6;

    public static double BEACON_ARM_LOAD_V3PO = 1, BEACON_ARM_STORAGE_V3PO = 0.6, BEACON_ARM_DELIVERY_V3PO = 0.8;

    public static double BATWING_DEPLOYED_V3PO = 0.58, BATWING_RETRACTED_V3PO = 0.8, BATWING_STORAGE_V3PO = 0.2, BATWING_DEPLOYED_LOW_JUNCTION_V3PO = 0.62;

}
