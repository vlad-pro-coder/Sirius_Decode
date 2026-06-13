package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class Ungureanu extends LinearOpMode {

    public static double power = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class,"motorsageata");
        Telemetry Dashtelemetry = FtcDashboard.getInstance().getTelemetry();


        waitForStart();

        while(opModeIsActive())
        {

            motor.setPower(power);

            Dashtelemetry.update();
            Dashtelemetry.addData("amps",motor.getCurrent(CurrentUnit.AMPS));
        }
    }
}
