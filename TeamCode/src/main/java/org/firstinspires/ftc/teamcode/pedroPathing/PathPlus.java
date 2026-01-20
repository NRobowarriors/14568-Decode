package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class PathPlus {
    public Path path;
    public boolean continueDriving;
    public PathChain pathChain;
    public PathPlus(Path pathIn, boolean continueDrivingIn, PathChain pathChainIn) {
        path = pathIn;
        continueDriving = continueDrivingIn;
        pathChain = pathChainIn;
    }
}
