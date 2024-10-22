package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

public class Robot {
    public Follower follower;
    public Intake intake;
    public Extendo extendo;
    public Lift lift;
    public Claw claw;


    public Robot(HardwareMap hwMap) {
        follower = new Follower(hwMap);
        intake = Intake.getInstance(hwMap);
        extendo = Extendo.getInstance(hwMap);
        lift = Lift.getInstance(hwMap);
        claw = Claw.getInstance(hwMap);
    }

}
