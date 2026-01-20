package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
public class PosePlus {
  public Pose pose;
  public boolean continueDriving;
  public PosePlus(Pose poseIn, boolean continueDrivingIn){
      pose = poseIn;
      continueDriving = continueDrivingIn;
  }
}
