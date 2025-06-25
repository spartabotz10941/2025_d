package OPs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Subsystems.ArmV2;
import Subsystems.DriveTrainV2;
import Subsystems.Hand;
import Subsystems.Lift;
import Subsystems.Supersystems;
@Config
@TeleOp
public class TeleOp2025V2 extends OpMode {
    public static DriveTrainV2 train ;
    public static Lift lift ;
    public static ArmV2 arm ;
    //public static Hand hand ;
    public static Supersystems supersystems;
    public double g1LeftStickY = 0.0;
    public double g1LeftStickX = 0.0;

    public double g1RightStickY = 0.0;
    public double g1RightStickX = 0.0;
    private boolean reset_is_done = false;
    public boolean finemovementbool;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();





    @Override
    public void init(){
        train = new DriveTrainV2(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new ArmV2 (hardwareMap);
        //hand = new Hand(hardwareMap);
        supersystems = new Supersystems(hardwareMap);

    }

    @Override
    public void init_loop(){

    }

    @Override
    public void loop(){
        if (gamepad2.left_stick_button) reset_is_done = !reset_is_done;
        if (!reset_is_done){
            //arm.shoulderReset();
            //lift.liftReset();
        }
        //Remi
        //if (gamepad2.y){lift.lift_state = 0;}
        //if (gamepad2.a){lift.lift_state = 1;}
        //lift.liftControlSwitch();
        //lift.liftMove((gamepad2.right_trigger-gamepad2.left_trigger));
        dashboardTelemetry.addData("triggers", gamepad2.right_trigger-gamepad2.left_trigger);







        //Linus
        if (gamepad1.right_trigger > 0){
            finemovementbool = true;
        } else if (gamepad1.right_trigger == 0) {
            finemovementbool = false;
        }
        train.finecontrols(finemovementbool);
        train.DriveCentric(gamepad1.left_stick_x * train.fine_move, gamepad1.left_stick_y * train.fine_move, gamepad1.right_stick_x);
        train.reset_odo(gamepad1.share);



        supersystems.update();

        dashboardTelemetry.update();

    }


    private void gamePadInput(){
        g1LeftStickY =gamepad1.left_stick_y *-1;
        g1LeftStickX =gamepad1.left_stick_x;

        g1RightStickY =gamepad1.right_stick_y *-1;
        g1RightStickX =gamepad1.right_stick_x;
    }
}
