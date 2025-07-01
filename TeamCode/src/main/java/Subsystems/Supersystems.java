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
    public  ArmV2 arm ;
    public  Hand hand ;

    public  Lift lift;

    public DriveTrainV2 drive;

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
        lift = new Lift(hardwareMap);
        drive = new DriveTrainV2(hardwareMap);


    }

    public void prepickup (){
        arm.setGearPositionForward();
        arm.setPulleyPositionForward();
        lift.setLiftPickUp();
        hand.movelrservo(0.5);
        hand.moveudservo(0.5);
        hand.rollers(0);
        drive.train_set_heading_submersible();

    }

    public void dropoff(){
        arm.setPulleyPositionBackward();
        arm.setGearPositionBackward();
        lift.setLiftDropoff();
        drive.train_set_heading_basket();
        hand.rollers(0);

        hand.moveudservo(0.5);
        hand.movelrservo(0.5);

    }

    public void armStarting(){
        arm.setGearPositionBackward();
        arm.setPulleyPositionForward();
    }
    public void pickup(){
        arm.setGearPositionForward();
        arm.setPulleyPositionForward();
        lift.setliftGround();
        hand.movelrservo(0.5);
        hand.moveudservo(0.5);
        hand.rollers(1);
        drive.train_set_heading_submersible();

    }
    public void release (){
        hand.rollers(-1);
        hand.moveudservo(0.8);
    }



    public void update(){
        arm.update();
        lift.update();
    }
     public boolean isAtState() {
         if (arm.isAtState() && lift.isAtState()) {
             return true;
         } else {
             return false;
         }
     }








}
