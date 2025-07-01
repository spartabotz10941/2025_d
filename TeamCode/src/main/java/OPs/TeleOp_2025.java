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

@TeleOp (name = "TeleOp 2025")
@Config

public class TeleOp_2025 extends OpMode{
    public static DriveTrainV2 train ;
    public static Lift lift ;
    public static ArmV2 arm ;
    public static Hand hand ;
    public static Supersystems supers;

    public double g1LeftStickY = 0.0;
    public double g1LeftStickX = 0.0;

    public double g1RightStickY = 0.0;
    public double g1RightStickX = 0.0;
    private boolean reset_is_done = false;
    public boolean finemovementbool = false;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();





    @Override
    public void init(){


        supers = new Supersystems(hardwareMap);

    }

    @Override
    public void init_loop(){

    }

    @Override
    public void loop(){
        if (gamepad1.a){//intake position
            supers.prepickup();
        }else if (gamepad1.b) { //output high basket
            supers.pickup();
        }else if (gamepad1.x) {
            supers.release();

        }else if (gamepad1.y){
            supers.dropoff();

        }else if (gamepad2.x){
            supers.release();

        }

        if (gamepad1.right_trigger > 0){
            finemovementbool = true;
        } else if (gamepad1.right_trigger == 0) {
            finemovementbool = false;
        }
        supers.drive.finecontrols(finemovementbool);


        supers.drive.DriveCentric(gamepad1.left_stick_x * supers.drive.fine_move, gamepad1.left_stick_y * supers.drive.fine_move, gamepad1.right_stick_x);
        supers.drive.reset_odo(gamepad1.share);




        supers.update();
        dashboardTelemetry.update();

    }


    private void gamePadInput(){
        g1LeftStickY =gamepad1.left_stick_y *-1;
        g1LeftStickX =gamepad1.left_stick_x;

        g1RightStickY =gamepad1.right_stick_y *-1;
        g1RightStickX =gamepad1.right_stick_x;
    }
}
