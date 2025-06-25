package Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Supersystems_opmode extends Supersystems{

    public int state = 0;

    public DriveTrainV2 drive;
    public Supersystems_opmode(@NonNull HardwareMap hardwareMap) {
        super(hardwareMap);
        drive = new DriveTrainV2(hardwareMap);


    }

    public void intake_sequence() {
        switch ( state){
            case 0:

                armForward();
                //lift.setLiftPickUp();
                hand.limelightStart(hand.limelightRed);
                if (hand.result.isValid()){
                    try {
                        Thread.sleep(3000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    state = 1;

                }


            case 1:
                arm.gearSetLimelightTarget(hand.getDistanceFromTy());
                drive.strafePid(hand.getDistanceFromTx());
                hand.turnToAngle();

                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                state = 2;
            case 2:
                //lift.setliftGround();
                hand.rollers(1);
                if (hand.distance() >= 4.1){
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    state = 3;

                }
            case 3:
                //lift.setLiftPickUp();
                armStarting();
        }
    }


}
