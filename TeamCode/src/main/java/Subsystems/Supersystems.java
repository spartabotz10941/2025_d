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
    public static Lift lift;
    public static ArmV2 arm ;
    public static Hand hand ;
    public static MyLimeLight myLimeLight;
    public int counter = 0;
    public double tx;
    public double ty;
    public Pose start_pose = new Pose(0,0);

    public Path path;
    public Follower follower;


    public Supersystems(@NonNull HardwareMap hardwareMap){
        lift = new Lift(hardwareMap);
        arm = new ArmV2 (hardwareMap);
        hand = new Hand(hardwareMap);
        myLimeLight = new MyLimeLight(hardwareMap);
        myLimeLight.start(myLimeLight.red);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
    }

    public void sampleDropoff(){
        lift.setLiftDropoff();
        arm.setPosition_Basket();
        hand.setWristDropoff();

    }

    public void samplePickUp (){
        lift.setLiftPickUp();
        arm.setPosition_PickUp();
        hand.setWristPickUp();
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        hand.setHandOpen();

    }

    public void specimenPickUp (){
        lift.setLiftPickUp();
        arm.setPosition_SpecimenPickUp();
        hand.setWristSpecimenPickUp();
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        hand.setHandOpen();

    }

    public void specimenHook(){
        lift.setLiftHook();
        arm.setPosition_Hook();
        hand.setWristMiddle();
        hand.setHandClose();
    }
    public boolean dropOffFinished(){
        boolean a = false;
        if (Math.abs(lift.getPosition() - Lift.lift_dropoff) < 15){
            a = true;
        }
        return a;
    }

    public void basePosition(){
        lift.setLiftPickUp();
        arm.setPositionBase();
        hand.setWristMiddle();
    }

    public void closeHand(){
        hand.setHandClose();
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void liftlift(){
        lift.setliftlift();
        arm.setPositionBase();

    }
    public void openHand(){
        hand.setHandOpen();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void centerOnSample(){
        myLimeLight.update();
        tx = myLimeLight.getTx();
        ty = myLimeLight.getTy();
        //follower.setStartingPose(start_pose);

        //Pose pose = new Pose(ty * 0.115,-tx * 0.225);

        //path= new Path(new BezierLine(new Point(0,0), new Point(pose)));
        //counter = counter + 1;
    }
    public void wristHookOn(){
        hand.setWristHook();
    }

    public void update() {
        arm.update();
        lift.update();
    }
}
