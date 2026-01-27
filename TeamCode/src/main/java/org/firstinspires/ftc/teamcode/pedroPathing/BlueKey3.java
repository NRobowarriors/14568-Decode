package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BlueKey3")

public class BlueKey3 extends LinearOpMode {

    @Override
    public void runOpMode() {
        AllAutoCode paths = new AllAutoCode(AutoEnum.BlueKey3, hardwareMap, this, telemetry);
        paths.runOpMode();
    }
}
