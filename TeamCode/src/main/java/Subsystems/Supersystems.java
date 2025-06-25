package Subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
public class Supersystems {
    //public static Lift lift;
    public static ArmV2 arm ;
    public static Hand hand ;

    public int counter = 0;
    public double tx;
    public double ty;
    public Pose start_pose = new Pose(0,0);

    public Path path;
    public Follower follower;


    public Supersystems(@NonNull HardwareMap hardwareMap){
        //lift = new Lift(hardwareMap);
        arm = new ArmV2 (hardwareMap);
        hand = new Hand(hardwareMap);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
    }

    public void armForward (){
        arm.setGearPositionForward();
        arm.setPulleyPositionForward();
    }

    public void armBackwards(){
        arm.setPulleyPositionBackward();
        arm.setGearPositionBackward();
    }

    public void armStarting(){
        arm.setGearPositionBackward();
        arm.setPulleyPositionBackward();
    }

    public void update(){
        arm.update();
        //lift.update();
    }








}
