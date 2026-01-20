package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedWall")

public class RedWall extends LinearOpMode {

    private AllAutoCode paths = new AllAutoCode(AutoEnum.RedWall, hardwareMap, this, telemetry);

    @Override
    public void runOpMode() {
        paths.runOpMode();
    }
}

