package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BlueWall")

public class BlueWall extends LinearOpMode {



    @Override
    public void runOpMode() {
        AllAutoCode paths = new AllAutoCode(AutoEnum.BlueWall, hardwareMap, this, telemetry);
        paths.runOpMode();
    }
}
