package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Dumper;
import org.firstinspires.ftc.teamcode.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class Robot {

    private static Intake intake;
    private static Extendo extendo;
    private static Lift lift;
    private static Claw claw;
    private static Dumper dumper;

    public static Intake getIntake(HardwareMap hwMap) {
        if (intake == null) {
            intake = new Intake(hwMap);
        }
        return intake;
    }

    public static Extendo getExtendo(HardwareMap hwMap) {
        if (extendo == null) {
            extendo = new Extendo(hwMap);
        }
        return extendo;
    }

    public static Lift getLift(HardwareMap hwMap) {
        if (lift == null) {
            lift = new Lift(hwMap);
        }
        return lift;
    }

    public static Claw getClaw(HardwareMap hwMap) {
        if (claw == null) {
            claw = new Claw(hwMap);
        }
        return claw;
    }

    public static Dumper getDumper(HardwareMap hwMap) {
        if (dumper == null) {
            dumper = new Dumper(hwMap);
        }
        return dumper;
    }
}
