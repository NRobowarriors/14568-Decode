package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedWall")

public class RedWall extends LinearOpMode {

    @Override
    public void runOpMode() {
        AllAutoCode paths = new AllAutoCode(AutoEnum.RedWall3, hardwareMap, this, telemetry);paths.runOpMode();
        paths.runOpMode();
    }
}

