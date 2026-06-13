package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Shooter;

@Config
@TeleOp
public class TestRPMWorks extends LinearOpMode {

    public static double power = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);

        while(opModeInInit()){
            RobotInitializers.clearCache();

            RobotInitializers.Dashtelemetry.addData("RPM test", Shooter.FlyWheelEncoder.getRPM());
            RobotInitializers.Dashtelemetry.addData("VEL test", Shooter.FlyWheelEncoder.getVelocity());
            Shooter.FlyWheelEncoder.setPower(power);


            RobotInitializers.Dashtelemetry.addData("RPM test raw", Shooter.FlyWheelEncoder.getRPM());
            RobotInitializers.Dashtelemetry.addData("VEL test raw", Shooter.FlyWheelEncoder.getVelocity());

        }
    }
}
