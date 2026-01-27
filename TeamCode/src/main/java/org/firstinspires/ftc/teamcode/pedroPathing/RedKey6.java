package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedKey6")

public class RedKey6 extends LinearOpMode {

    @Override
    public void runOpMode() {
        AllAutoCode paths = new AllAutoCode(AutoEnum.RedKey6, hardwareMap, this, telemetry);
        paths.runOpMode();
    }
}
