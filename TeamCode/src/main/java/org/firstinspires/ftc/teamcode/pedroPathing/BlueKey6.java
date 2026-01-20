package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BlueKey6")

public class BlueKey6 extends LinearOpMode {

    private AllAutoCode paths = new AllAutoCode(AutoEnum.BlueKey6, hardwareMap, this, telemetry);

    @Override
    public void runOpMode() {
        paths.runOpMode();
    }
}
