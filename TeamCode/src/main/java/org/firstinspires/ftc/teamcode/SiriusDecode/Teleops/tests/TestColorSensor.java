package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.RGBsensor;

@TeleOp
@Config
public class TestColorSensor extends LinearOpMode {

    public static String namecolorsensor = "colorsensor";

    @Override
    public void runOpMode() throws InterruptedException {

        RobotInitializers.InitializeFull(hardwareMap);
        waitForStart();

        while(opModeIsActive()){

            RobotInitializers.clearCache();

            RGBsensor color = hardwareMap.get(RGBsensor.class,namecolorsensor);

            RobotInitializers.Dashtelemetry.addData("r",color.RGB.R);
            RobotInitializers.Dashtelemetry.addData("g",color.RGB.G);
            RobotInitializers.Dashtelemetry.addData("b",color.RGB.B);

            RobotInitializers.Dashtelemetry.addData("d",color.RGB.D);

            RobotInitializers.Dashtelemetry.addData("color seen", color.getColorSeenBySensor());

            RobotInitializers.Dashtelemetry.update();
        }

    }
}
