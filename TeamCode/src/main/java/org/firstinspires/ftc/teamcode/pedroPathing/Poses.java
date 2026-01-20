package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class Poses {
    public Poses (AutoEnum auto, Follower followerIn) {
        follower = followerIn;
        switch (auto)
        {
            case BlueWall:
                createPaths(BlueWallPoses);
                break;
            case BlueWall3:
                createPaths(BlueWall3Poses);
                break;
            case BlueWall6:
                createPaths(BlueWall6Poses);
                break;
            case BlueWall9:
                createPaths(BlueWall9Poses);
                break;
            case RedWall:
                createPaths(RedWallPoses);
                break;
            case RedWall3:
                createPaths(RedWall3Poses);
                break;
            case RedWall6:
                createPaths(RedWall6Poses);
                break;
            case RedWall9:
                createPaths(RedWall9Poses);
                break;
            case BlueKey:
                createPaths(BlueKeyPoses);
                break;
            case BlueKey3:
                createPaths(BlueKey3Poses);
                break;
            case BlueKey6:
                createPaths(BlueKey6Poses);
                break;
            case BlueKey9:
                createPaths(BlueKey9Poses);
                break;
            case RedKey:
                createPaths(RedKeyPoses);
                break;
            case RedKey3:
                createPaths(RedKey3Poses);
                break;
            case RedKey6:
                createPaths(RedKey6Poses);
                break;
            case RedKey9:
                createPaths(RedKey9Poses);


        }
    }
    private Follower follower;
    public Path startingPath;
    public PathPlus[] pathPlus;
    public PathChain[] PathChains;
    private PosePlus[] BlueWallPoses = new PosePlus[]{
            new PosePlus(new Pose(18, 128, Math.toRadians(145)), false),//starting pose
            new PosePlus(new Pose(60, 90, Math.toRadians(135)), false),//scoring pose
            new PosePlus(new Pose(49, 81, Math.toRadians(0)), true),
            new PosePlus(new Pose(18, 81, Math.toRadians(0)), true),
            new PosePlus(new Pose(60, 90, Math.toRadians(135)), false),//scoring pose
            new PosePlus(new Pose(49, 57, Math.toRadians(0)), true),
            new PosePlus(new Pose(12, 57, Math.toRadians(0)), true),
            new PosePlus(new Pose(62, 120, Math.toRadians(135)), false),//scoring pose
            new PosePlus(new Pose(49, 33, Math.toRadians(0)), true),
            new PosePlus(new Pose(10, 33, Math.toRadians(0)), true),
            new PosePlus(new Pose(60, 9, Math.toRadians(90)), false), //score pose
            new PosePlus(new Pose(60, 70, Math.toRadians(135)), false),//scoring pose
            new PosePlus(new Pose(55, 40, Math.toRadians(90)), false), //move off line
    } ;
    private PosePlus[] BlueWall3Poses = new PosePlus[]{
            new PosePlus(new Pose(18, 128, Math.toRadians(145)), false),//starting pose
            new PosePlus(new Pose(60, 90, Math.toRadians(135)), false),//scoring pose
            new PosePlus(new Pose(55, 40, Math.toRadians(90)), false), //move off line
    } ;
    private PosePlus[] BlueWall6Poses = new PosePlus[]{
            new PosePlus(new Pose(18, 128, Math.toRadians(145)), false),//starting pose
            new PosePlus(new Pose(60, 90, Math.toRadians(135)), false),//scoring pose
            new PosePlus(new Pose(49, 81, Math.toRadians(0)), true),
            new PosePlus(new Pose(18, 81, Math.toRadians(0)), true),
            new PosePlus(new Pose(60, 90, Math.toRadians(135)), false),//scoring pose
            new PosePlus(new Pose(55, 40, Math.toRadians(90)), false), //move off line
    } ;
    private PosePlus[] BlueWall9Poses = new PosePlus[]{
            new PosePlus(new Pose(18, 128, Math.toRadians(145)), false),//starting pose
            new PosePlus(new Pose(60, 90, Math.toRadians(135)), false),//scoring pose
            new PosePlus(new Pose(49, 81, Math.toRadians(0)), true),
            new PosePlus(new Pose(18, 81, Math.toRadians(0)), true),
            new PosePlus(new Pose(60, 90, Math.toRadians(135)), false),//scoring pose
            new PosePlus(new Pose(49, 57, Math.toRadians(0)), true),
            new PosePlus(new Pose(12, 57, Math.toRadians(0)), true),
            new PosePlus(new Pose(62, 120, Math.toRadians(135)), false),//scoring pose
            new PosePlus(new Pose(55, 40, Math.toRadians(90)), false), //move off line
    } ;
    private PosePlus[] RedWallPoses = new PosePlus[]{
            new PosePlus(new Pose(120, 128, Math.toRadians(35)), false),//starting pose
            new PosePlus(new Pose(100, 128, Math.toRadians(35)), false),
            new PosePlus(new Pose(78, 80, Math.toRadians(45)), false),//scoring pose
            new PosePlus(new Pose(120, 83, Math.toRadians(180)), true),
            new PosePlus(new Pose(132, 85, Math.toRadians(180)), true),
            new PosePlus(new Pose(78, 80, Math.toRadians(45)), false),//scoring pose
            new PosePlus(new Pose(120, 59, Math.toRadians(180)), true),
            new PosePlus(new Pose(132, 61, Math.toRadians(180)), true),
            new PosePlus(new Pose(78, 80, Math.toRadians(45)), false),//scoring pose
            new PosePlus(new Pose(120, 35, Math.toRadians(180)), true),
            new PosePlus(new Pose(132, 37, Math.toRadians(180)), true),
            new PosePlus(new Pose(78, 80, Math.toRadians(45)), false),//scoring pose
           new PosePlus(new Pose(90, 120, Math.toRadians(90)), false)//move off line
    } ;
    private PosePlus[] RedWall3Poses = new PosePlus[]{
            new PosePlus(new Pose(120, 128, Math.toRadians(35)), false),//starting pose
            new PosePlus(new Pose(100, 128, Math.toRadians(35)), false),
            new PosePlus(new Pose(78, 80, Math.toRadians(45)), false),//scoring pose
            new PosePlus(new Pose(90, 120, Math.toRadians(90)), false)//move off line
    } ;
    private PosePlus[] RedWall6Poses = new PosePlus[]{
            new PosePlus(new Pose(120, 128, Math.toRadians(35)), false),//starting pose
            new PosePlus(new Pose(100, 128, Math.toRadians(35)), false),
            new PosePlus(new Pose(78, 80, Math.toRadians(45)), false),//scoring pose
            new PosePlus(new Pose(120, 83, Math.toRadians(180)), true),
            new PosePlus(new Pose(132, 85, Math.toRadians(180)), true),
            new PosePlus(new Pose(78, 80, Math.toRadians(45)), false),//scoring pose
            new PosePlus(new Pose(90, 120, Math.toRadians(90)), false)//move off line
    } ;
    private PosePlus[] RedWall9Poses = new PosePlus[]{
            new PosePlus(new Pose(120, 128, Math.toRadians(35)), false),//starting pose
            new PosePlus(new Pose(100, 128, Math.toRadians(35)), false),
            new PosePlus(new Pose(78, 80, Math.toRadians(45)), false),//scoring pose
            new PosePlus(new Pose(120, 83, Math.toRadians(180)), true),
            new PosePlus(new Pose(132, 85, Math.toRadians(180)), true),
            new PosePlus(new Pose(78, 80, Math.toRadians(45)), false),//scoring pose
            new PosePlus(new Pose(120, 59, Math.toRadians(180)), true),
            new PosePlus(new Pose(132, 61, Math.toRadians(180)), true),
            new PosePlus(new Pose(78, 80, Math.toRadians(45)), false),//scoring pose
            new PosePlus(new Pose(90, 120, Math.toRadians(90)), false)//move off line
    } ;
    private PosePlus[] BlueKeyPoses = new PosePlus[]{
            new PosePlus(new Pose(60, 9, Math.toRadians(90)), false), //starting pose
            new PosePlus(new Pose(60, 12, Math.toRadians(115)), false),//scoring pose
            new PosePlus(new Pose(49, 33, Math.toRadians(0)), true),
            new PosePlus(new Pose(10, 33, Math.toRadians(0)), true),
            new PosePlus(new Pose(60, 10, Math.toRadians(115)), false),//scoring pose
            new PosePlus(new Pose(49, 57, Math.toRadians(0)), true),
            new PosePlus(new Pose(12, 57, Math.toRadians(0)), true),
            new PosePlus(new Pose(60, 10, Math.toRadians(115)), false),//scoring pose
            new PosePlus(new Pose(49, 81, Math.toRadians(0)), true),
            new PosePlus(new Pose(18, 81, Math.toRadians(0)), true),
            new PosePlus(new Pose(60, 10, Math.toRadians(115)), false),//scoring pose
            new PosePlus(new Pose(55, 40, Math.toRadians(90)), false) //move off line
    } ;
    private PosePlus[] BlueKey3Poses = new PosePlus[]{
            new PosePlus(new Pose(60, 9, Math.toRadians(90)), false), //starting pose
            new PosePlus(new Pose(60, 10, Math.toRadians(115)), false),//scoring pose
            new PosePlus(new Pose(55, 40, Math.toRadians(90)), false) //move off line
    } ;
    private PosePlus[] BlueKey6Poses = new PosePlus[]{
            new PosePlus(new Pose(60, 9, Math.toRadians(90)), false), //starting pose
            new PosePlus(new Pose(60, 12, Math.toRadians(115)), false),//scoring pose
            new PosePlus(new Pose(49, 33, Math.toRadians(0)), true),
            new PosePlus(new Pose(10, 33, Math.toRadians(0)), true),
            new PosePlus(new Pose(60, 10, Math.toRadians(115)), false),//scoring pose
            new PosePlus(new Pose(55, 40, Math.toRadians(90)), false) //move off line
    } ;
    private PosePlus[] BlueKey9Poses = new PosePlus[]{
            new PosePlus(new Pose(60, 9, Math.toRadians(90)), false), //starting pose
            new PosePlus(new Pose(60, 12, Math.toRadians(115)), false),//scoring pose
            new PosePlus(new Pose(49, 33, Math.toRadians(0)), true),
            new PosePlus(new Pose(10, 33, Math.toRadians(0)), true),
            new PosePlus(new Pose(60, 10, Math.toRadians(115)), false),//scoring pose
            new PosePlus(new Pose(49, 57, Math.toRadians(0)), true),
            new PosePlus(new Pose(12, 57, Math.toRadians(0)), true),
            new PosePlus(new Pose(60, 10, Math.toRadians(115)), false),//scoring pose
            new PosePlus(new Pose(55, 40, Math.toRadians(90)), false) //move off line
    } ;
    private PosePlus[] RedKeyPoses = new PosePlus[]{
            new PosePlus(new Pose(84, 9, Math.toRadians(90)), false),//starting pose
            new PosePlus(new Pose(84, 10, Math.toRadians(70)), false),//scoring pose
            new PosePlus( new Pose(95, 39, Math.toRadians(180)), true),
            new PosePlus(new Pose(129, 41, Math.toRadians(180)), true),
            new PosePlus( new Pose(84, 10, Math.toRadians(70)), false),//scoring pose
            new PosePlus( new Pose(95, 63, Math.toRadians(180)), true),
            new PosePlus( new Pose(130, 65, Math.toRadians(180)), true),
            new PosePlus( new Pose(84, 10, Math.toRadians(70)), false),//scoring pose
            new PosePlus( new Pose(95, 87, Math.toRadians(180)), true),
            new PosePlus( new Pose(120, 85, Math.toRadians(180)), true),
            new PosePlus( new Pose(84, 10, Math.toRadians(70)), false),//scoring pose
            new PosePlus( new Pose(84, 40, Math.toRadians(90)), false) //move off line
    } ;
    private PosePlus[] RedKey3Poses = new PosePlus[]{
            new PosePlus(new Pose(84, 9, Math.toRadians(90)), false),//starting pose
            new PosePlus(new Pose(84, 10, Math.toRadians(70)), false),//scoring pose
            new PosePlus( new Pose(84, 40, Math.toRadians(90)), false) //move off line
    } ;
    private PosePlus[] RedKey6Poses = new PosePlus[]{
            new PosePlus(new Pose(84, 9, Math.toRadians(90)), false),//starting pose
            new PosePlus(new Pose(84, 10, Math.toRadians(70)), false),//scoring pose
            new PosePlus( new Pose(95, 39, Math.toRadians(180)), true),
            new PosePlus(new Pose(129, 41, Math.toRadians(180)), true),
            new PosePlus( new Pose(84, 10, Math.toRadians(70)), false),//scoring pose
            new PosePlus( new Pose(84, 40, Math.toRadians(90)), false) //move off line
    } ;
    private PosePlus[] RedKey9Poses = new PosePlus[]{
            new PosePlus(new Pose(84, 9, Math.toRadians(90)), false),//starting pose
            new PosePlus(new Pose(84, 10, Math.toRadians(70)), false),//scoring pose
            new PosePlus( new Pose(95, 39, Math.toRadians(180)), true),
            new PosePlus(new Pose(129, 41, Math.toRadians(180)), true),
            new PosePlus( new Pose(84, 10, Math.toRadians(70)), false),//scoring pose
            new PosePlus( new Pose(95, 63, Math.toRadians(180)), true),
            new PosePlus( new Pose(130, 65, Math.toRadians(180)), true),
            new PosePlus( new Pose(84, 10, Math.toRadians(70)), false),//scoring pose
            new PosePlus( new Pose(84, 40, Math.toRadians(90)), false) //move off line
    } ;

    private void createPaths(PosePlus[] poses){
        follower.setStartingPose(poses[0].pose);

        pathPlus = new PathPlus[poses.length-1];
        for (int i = 0; i <= poses.length-2; i++)
        {
            if (i == 0){
                startingPath = new Path(new BezierLine(poses[0].pose, poses[1].pose));
                startingPath.setLinearHeadingInterpolation(poses[0].pose.getHeading(), poses[1].pose.getHeading());
                pathPlus[i] = new PathPlus(startingPath, poses[i+1].continueDriving, null);
            }
            else {
                pathPlus[i] = new PathPlus(null, poses[i+1].continueDriving, follower.pathBuilder().
                        addPath(new BezierLine(poses[i].pose, poses[i+1].pose))
                        .setLinearHeadingInterpolation(poses[i].pose.getHeading(), poses[i+1].pose.getHeading(), 0.8)
                        .build());
            }
        }
    }
}

