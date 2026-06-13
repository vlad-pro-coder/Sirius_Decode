package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class TestLocalization extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            RobotInitializers.clearCache();

            RobotInitializers.Dashtelemetry.addData("x", Localizer.getCurrentPosition().x);
            RobotInitializers.Dashtelemetry.addData("y", Localizer.getCurrentPosition().y);
            RobotInitializers.Dashtelemetry.addData("h", Localizer.getCurrentPosition().h);

            RobotInitializers.Dashtelemetry.addData("xencoder", Localizer.pinPoint.getEncoderX());
            RobotInitializers.Dashtelemetry.addData("yencoder", Localizer.pinPoint.getEncoderY());
            RobotInitializers.Dashtelemetry.addData("freq", Localizer.pinPoint.getFrequency());
            RobotInitializers.Dashtelemetry.addData("angle row",Localizer.pinPoint.getHeading(AngleUnit.DEGREES));

            Chassis.drive(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_trigger - gamepad1.left_trigger
            );

            //Localizer.pinPoint.setOffsets(Localizer.X,Localizer.Y, DistanceUnit.MM);

            Localizer.Update();

        }
    }
}