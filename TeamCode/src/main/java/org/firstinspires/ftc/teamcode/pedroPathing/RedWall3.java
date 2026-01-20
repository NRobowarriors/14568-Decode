package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedWall3")

public class RedWall3 extends LinearOpMode {

    private AllAutoCode paths = new AllAutoCode(AutoEnum.RedWall3, hardwareMap, this, telemetry);

    @Override
    public void runOpMode() {
        paths.runOpMode();
    }
}

