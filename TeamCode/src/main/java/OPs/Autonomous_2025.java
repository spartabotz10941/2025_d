package OPs;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import Subsystems.ArmV2;
import Subsystems.DriveTrainV2;
import Subsystems.Lift;
import Subsystems.Supersystems;

@Autonomous(name = "Autonomous 2025")
@Config
public class Autonomous_2025 extends OpMode {


    Supersystems supersystems ;


    private int caseselector = 0;
    private int centeringselector = 0;

    @Override
    public void init(){
        supersystems = new Supersystems(hardwareMap);


    }

    @Override
    public void init_loop(){
        //arm.shoulderReset();
        //lift.liftReset();
    }

    @Override
    public void loop(){



    }
    /*
    private void switchStatement(){
        switch (caseselector){
            case 0:
                supersystems.();
                try {
                    Thread.sleep(4000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                if(){caseselector = 1;}
            case 1:
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                caseselector = 2;
            case 2:

        }
    }



    private void centeringSwitch (){
        switch (centeringselector){
            case 0:

                if (supersystems.counter > 10){centeringselector = 1;}
            case 1:
                supersystems.follower.update();
                if (supersystems.ty < 1 ){centeringselector = 2;}
            case 2:
                //supersystems.samplePickUp();
        }
    }
    */
}
