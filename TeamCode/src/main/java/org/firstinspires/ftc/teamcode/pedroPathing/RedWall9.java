package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedWall9")

public class RedWall9 extends LinearOpMode {

    @Override
    public void runOpMode() {
        AllAutoCode paths = new AllAutoCode(AutoEnum.RedWall9, hardwareMap, this, telemetry);
        paths.runOpMode();
    }
}

