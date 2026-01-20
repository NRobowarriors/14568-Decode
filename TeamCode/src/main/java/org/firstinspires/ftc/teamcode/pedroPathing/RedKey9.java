package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedKey9")

public class RedKey9 extends LinearOpMode {

    private AllAutoCode paths = new AllAutoCode(AutoEnum.RedKey9, hardwareMap, this, telemetry);

    @Override
    public void runOpMode() {
        paths.runOpMode();
    }
}
