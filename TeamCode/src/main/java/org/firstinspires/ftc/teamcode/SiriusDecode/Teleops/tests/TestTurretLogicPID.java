package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Turret;

@Config
@TeleOp
public class TestTurretLogicPID extends LinearOpMode {

    public static boolean Setangleyes = true;
    public static double angletogive = 0;

    public static double a = 0,v = 0,d = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotInitializers.InitializeFull(hardwareMap);

        Turret turret = new Turret();

        waitForStart();

        while(opModeIsActive())
        {
            RobotInitializers.clearCache();

            Localizer.Update();

            if(!Setangleyes)
                turret.updateTurret(Localizer.getCurrentPosition());
            else {
                turret.setAngle(angletogive);
            }
        }
    }
}
