package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedKey6")

public class RedKey6 extends LinearOpMode {

    private AllAutoCode paths = new AllAutoCode(AutoEnum.RedKey6, hardwareMap, this, telemetry);

    @Override
    public void runOpMode() {
        paths.runOpMode();
    }
}
