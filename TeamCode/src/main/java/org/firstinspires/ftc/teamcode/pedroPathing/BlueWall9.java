package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BlueWall9")

public class BlueWall9 extends LinearOpMode {

    @Override
    public void runOpMode() {
        AllAutoCode paths = new AllAutoCode(AutoEnum.BlueWall9, hardwareMap, this, telemetry);
        paths.runOpMode();
    }
}
