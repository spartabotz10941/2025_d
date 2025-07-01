package OPs;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import Subsystems.Lift;
import Subsystems.Supersystems;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;






    /**
     * This is an example auto that showcases movement and control of two servos autonomously.
     * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
     * There are examples of different ways to build paths.
     * A path progression method has been created and can advance based on time, position, or other factors.
     *
     * @author Baron Henderson - 20077 The Indubitables
     * @version 2.0, 11/28/2024
     */

    @Config
    @Autonomous
    public class AutoBlueSpecimen extends OpMode {

        Supersystems supersystems;

        //this section allows us to access telemetry data from a browser
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        ElapsedTime clipTimer = new ElapsedTime();

        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;

        /** This is the variable where we store the state of our auto.
         * It is used by the pathUpdate method. */
        private int pathState;

        /* Create and Define Poses + Paths
         * Poses are built with three constructors: x, y, and heading (in Radians).
         * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
         * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
         */

        private final Pose startpose = new Pose(8,80,Math.toRadians(0));
        private final Pose clippose = new Pose(39,80,Math.toRadians(0));
        private final Pose pickup1pose = new Pose(30,120,Math.toRadians(0));
        private final Pose pickup2pose = new Pose(30,130,Math.toRadians(0));
        private final Pose pickup3pose = new Pose(30,130,Math.toRadians(30));
        private final Pose parkpose = new Pose(63,98,Math.toRadians(-90));
        private final Pose parkcontrolpose = new Pose(61.92,98,Math.toRadians(90));
        private final Pose scorepose = new Pose(61.92,98,Math.toRadians(90));



        /* These are our Paths and PathChains that we will define in buildPaths() */
        private PathChain clipline, pickup1line,score1line, pickup2line,score2line, pickup3line, score3line, parkline;

        /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
         * It is necessary to do this so that all the paths are built before the auto starts. **/
        public void buildPaths() {

            /* There are two major types of paths components: BezierCurves and BezierLines.
             *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
             *    - Control points manipulate the curve between the start and end points.
             *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
             *    * BezierLines are straight, and require 2 points. There are the start and end points.
             * Paths have can have heading interpolation: Constant, Linear, or Tangential
             *    * Linear heading interpolation:
             *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
             *    * Constant Heading Interpolation:
             *    - Pedro will maintain one heading throughout the entire path.
             *    * Tangential Heading Interpolation:
             *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
             * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
             * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

            /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
            clipline = follower.pathBuilder()
                    .addPath(
                            // Line 1
                            new BezierLine(
                                    new Point(startpose),
                                    new Point(clippose)
                            )
                    )
                    .setLinearHeadingInterpolation(startpose.getHeading(), clippose.getHeading())
                    .build();

            pickup1line = follower.pathBuilder()
                    .addPath(

                            new BezierLine(
                                    new Point(clippose),
                                    new Point(pickup1pose)
                            )
                    )
                    .setLinearHeadingInterpolation(clippose.getHeading(), pickup1pose.getHeading())
                    .build();


            score1line = follower.pathBuilder()
                    .addPath(

                            new BezierLine(
                                    new Point(pickup1pose),
                                    new Point(scorepose)
                            )
                    )
                    .setLinearHeadingInterpolation(pickup1pose.getHeading(), scorepose.getHeading())
                    .build();
            pickup2line = follower.pathBuilder()
                    .addPath(

                            new BezierLine(
                                    new Point(scorepose),
                                    new Point(pickup2pose)
                            )
                    )
                    .setLinearHeadingInterpolation(scorepose.getHeading(), pickup2pose.getHeading())
                    .build();
            score2line = follower.pathBuilder()
                    .addPath(

                            new BezierLine(
                                    new Point(pickup2pose),
                                    new Point(scorepose)
                            )
                    )
                    .setLinearHeadingInterpolation(pickup2pose.getHeading(), scorepose.getHeading())
                    .build();
            pickup3line= follower.pathBuilder()
                    .addPath(

                            new BezierLine(
                                    new Point(scorepose),
                                    new Point(pickup3pose)
                            )
                    )
                    .setLinearHeadingInterpolation(scorepose.getHeading(), pickup3pose.getHeading())
                    .build();
            score3line = follower.pathBuilder()
                    .addPath(

                            new BezierLine(
                                    new Point(pickup3pose),
                                    new Point(scorepose)
                            )
                    )
                    .setLinearHeadingInterpolation(pickup3pose.getHeading(), scorepose.getHeading())
                    .build();
            parkline = follower.pathBuilder()
                    .addPath(

                            new BezierCurve(
                                    new Point(scorepose),
                                    new Point(parkcontrolpose),
                                    new Point (parkpose)
                            )
                    )
                    .setLinearHeadingInterpolation(scorepose.getHeading(), parkpose.getHeading())
                    .build();

            /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */


        }

        /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
         * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
         * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
        public void autonomousPathUpdate() {
            switch (pathState) {
                case 0:
                    follower.followPath(clipline, true);
                    setPathState(1);
                    // clip preset

                    break;

                case 1:
                    if(!follower.isBusy()) {
                        follower.followPath(pickup1line, true);

                        //pickup preset
                        setPathState(2);
                    }
                    break;
                case 2:
                    //pick up
                    setPathState(3);

                    break;
                case 3:
                    if(!follower.isBusy()) {
                        follower.followPath(score1line, true);
                        // drop off
                        setPathState(4);
                    }
                    break;
                case 4:
                    // dropoff
                    setPathState(5);

                    break;
                case 5:
                    if(!follower.isBusy()) {
                        // pickup
                        follower.followPath(pickup2line, true);
                        setPathState(6);
                    }
                    break;
                case 6:
                    //pick up
                    setPathState(7);

                    break;
                case 7:
                    if(!follower.isBusy()) {
                        follower.followPath(score2line, true);
                        // drop off
                        setPathState(8);
                    }
                    break;
                case 8:
                    //drop off
                    setPathState(9);

                    break;
                case 9:
                    if(!follower.isBusy()) {
                        follower.followPath(pickup3line, true);
                        // pcik up
                        setPathState(10);
                    }
                    break;
                case 10:
                    //pick up
                    setPathState(11);

                    break;
                case 11:
                    if(!follower.isBusy()) {
                        follower.followPath(score3line, true);
                        //drop off
                        setPathState(12);
                    }
                    break;
                case 12:
                    //drop off
                    setPathState(13);

                    break;
                case 13:
                    if(!follower.isBusy()) {
                        follower.followPath(parkline, true);
                        //touch structure

                    }
                    break;

            }
        }

        /** These change the states of the paths and actions
         * It will also reset the timers of the individual switches **/
        public void setPathState(int pState) {
            pathState = pState;

        }

        /** This method is called once at the init of the OpMode. **/
        @Override
        public void init() {
            supersystems = new Supersystems(hardwareMap);

            Constants.setConstants(FConstants.class, LConstants.class);
            follower = new Follower(hardwareMap);
            follower.setStartingPose(new Pose(8.5, 81,Math.toRadians(0)));
            buildPaths();
        }

        /** This method is called continuously after Init while waiting for "play". **/
        @Override
        public void init_loop() {}

        /** This method is called once at the start of the OpMode.
         * It runs all the setup actions, including building paths and starting the path system **/
        @Override
        public void start() {
            setPathState(0);
            clipTimer.reset();
        }

        /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
        @Override
        public void loop() {
            follower.update();
            autonomousPathUpdate();
            supersystems.update();
            dashboardTelemetry.addData("stage",pathState);
            dashboardTelemetry.update();

        }



        /** We do not use this because everything should automatically disable **/
        @Override
        public void stop() {
        }
    }


