package org.firstinspires.ftc.teamcode.SiriusDecode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SiriusDecode.MathHelpers.Colors;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Shooter;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Storage;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Turret;
import org.firstinspires.ftc.teamcode.SiriusDecode.TeleopLogic.MainHandler;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class TeleopsStarter {
    public static TEAM team = TEAM.RED;
    public static Gamepad gm1;
    public static Gamepad gm2;
    public static double rot = 1;
    public static boolean reverse = true;

    public static double Hz = 30;
    public ElapsedTime timer;

    public static double AngleOffsetForDriving = -Math.PI/2;

    public MainHandler mainHandler;
    public static Storage storage;
    public static Shooter shooter;
    public static Turret turret;
    public static SparkFunOTOS.Pose2D resetPosition = new SparkFunOTOS.Pose2D(0,0,0);

    public static Intake intake;

    public static boolean DrivingFieldCentric = false;


    public static Gamepad prevgm1,prevgm2;

    public TeleopsStarter(HardwareMap hardwareMap,TEAM Selectedteam){

        team = Selectedteam;

        TeleopsStarter.gm1 = new Gamepad();
        TeleopsStarter.gm2 = new Gamepad();

        prevgm1 = new Gamepad();
        prevgm2 = new Gamepad();

        RobotInitializers.InitializeFull(hardwareMap);
        RobotInitializers.disable();
        intake = new Intake();
        storage = new Storage(intake);
        shooter = new Shooter();
        turret = new Turret();

        timer = new ElapsedTime();
        //aprilTagHandler = new AprilTagVision(hardwareMap,"Webcam 1");

        //aprilTagHandler.stop();

        /*Storage.Motif = new ArrayList<>(
                Arrays.asList(
                        Colors.ColorType.PURPLE,
                        Colors.ColorType.PURPLE,
                        Colors.ColorType.PURPLE
                )
        );*/


        mainHandler = new MainHandler();
    }

    public double getPowerSigned(double h, double p){
        double sgn = Math.signum(h);
        h = Math.abs(h);
        for(int i = 1; i < p; i++){
            h *= h;
        }
        return sgn * h;
    }

    public void InitUpdate(){
        RobotInitializers.clearCache();

        if(gm1.cross){
            DrivingFieldCentric = true;
        }
        prevgm1.copy(gm1);
        prevgm2.copy(gm2);
        Localizer.Update();
    }

    public void update(){

        RobotInitializers.clearCache(false);



            /*if (!gm2.square) {
                if (gm2.dpad_up) {
                    Storage.Motif = new ArrayList<>(
                            Arrays.asList(
                                    Colors.ColorType.GREEN,
                                    Colors.ColorType.PURPLE,
                                    Colors.ColorType.PURPLE
                            )
                    );
                    Storage.sort = true;
                }
                else if (gm2.dpad_right)
                {
                    Storage.Motif = new ArrayList<>(
                            Arrays.asList(
                                    Colors.ColorType.PURPLE,
                                    Colors.ColorType.GREEN,
                                    Colors.ColorType.PURPLE
                            )
                    );

                Storage.sort = true;
            }
                else if (gm2.dpad_left)
                {
                    Storage.Motif = new ArrayList<>(
                            Arrays.asList(
                                    Colors.ColorType.PURPLE,
                                    Colors.ColorType.PURPLE,
                                    Colors.ColorType.GREEN
                            )
                    );
                    Storage.sort = true;
                }
                else if (gm2.dpad_down) {
                    Storage.Motif = new ArrayList<>(
                            Arrays.asList(
                                    Colors.ColorType.PURPLE,
                                    Colors.ColorType.PURPLE,
                                    Colors.ColorType.PURPLE
                            )
                    );
                    Storage.sort = false;
                }
            }*/

                if (!DrivingFieldCentric) {
                    Chassis.drive(
                            (reverse ? -1 : 1) * getPowerSigned(gm1.left_stick_x, 3),
                            (reverse ? 1 : -1) * getPowerSigned(gm1.left_stick_y, 3),
                            -gm1.right_stick_x * rot
                    );
                } else
                    Chassis.driveFieldCentric(
                            (reverse ? -1 : 1) * gm1.left_stick_x,
                            (reverse ? 1 : -1) * gm1.left_stick_y,
                            -gm1.right_stick_x,
                            AngleOffsetForDriving
                    );





            storage.update();




        mainHandler.update();
        if(timer.seconds() >= 1.0/Hz) {
            timer.reset();
            shooter.updateFlyWheelSpeed();
            shooter.UpdateTurretAndPitch();
            turret.updateTurret(Localizer.getCurrentPosition());
        }
//            turret.setAngle(220);



        //shooter.GenerateOffsetFromVelocity();

        Localizer.Update();
        //RobotInitializers.Dashtelemetry.addData("bearing",aprilTagHandler.GetTeamTagBearing());
        prevgm1.copy(gm1);
        prevgm2.copy(gm2);
    }



}
