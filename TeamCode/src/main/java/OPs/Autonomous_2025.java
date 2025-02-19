package OPs;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import Subsystems.Arm;
import Subsystems.Lift;

@Autonomous(name = "Autonomous 2025")
@Config
public class Autonomous_2025 extends OpMode {
    Arm arm = new Arm();
    Lift lift = new Lift();

    @Override
    public void init(){

    }

    @Override
    public void init_loop(){
        arm.shoulderReset();
        lift.liftReset();
    }

    @Override
    public void loop(){
    }

}
