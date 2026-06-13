package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class ReadPotentiometers extends LinearOpMode {

    public static int port = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeHubs(hardwareMap);

        AnalogInput analog;


        waitForStart();

        while(opModeIsActive()){
            RobotInitializers.clearCache();

            analog = new AnalogInput(RobotInitializers.ControlHubAnalog,port);

            RobotInitializers.Dashtelemetry.addData("analogRead",analog.getVoltage());

            RobotInitializers.Dashtelemetry.update();


            }
    }
}
