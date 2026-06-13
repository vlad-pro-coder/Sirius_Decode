package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Shooter;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Turret;

@TeleOp
@Config
public class CalibTurret extends LinearOpMode {

    public static double angleToInit = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);

        Turret turret = new Turret();

        waitForStart();

        while(opModeIsActive())
        {
            RobotInitializers.clearCache();

            turret.setAngleToInit(angleToInit);
        }
    }
}
