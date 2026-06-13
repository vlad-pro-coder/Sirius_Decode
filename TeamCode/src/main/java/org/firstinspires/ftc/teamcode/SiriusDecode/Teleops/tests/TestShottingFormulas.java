package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.SiriusDecode.ActionsCommandLineImplementation.AsyncScheduler;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Shooter;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.ShooterCalculator;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Storage;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Turret;


@Config
@TeleOp
public class TestShottingFormulas extends LinearOpMode {


    public static Intake intake;
    public static Storage storage;
    public static Shooter shooter;


    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);

        shooter = new Shooter();
        intake = new Intake();
        storage = new Storage(intake);
        ShooterCalculator formulas = new ShooterCalculator();

        AsyncScheduler tasks = new AsyncScheduler();

        while(opModeInInit()){
            RobotInitializers.clearCache();
        }

        waitForStart();

        while(opModeIsActive()){
            RobotInitializers.clearCache();

            RobotInitializers.Dashtelemetry.addData("Distance to goal",Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(), Turret.goalPosition));

            ShooterCalculator.ShootData data = formulas.CalculateShootData(shooter.RPMError());

            RobotInitializers.Dashtelemetry.addData("hood angle",data.hoodAngleDegrees);
            RobotInitializers.Dashtelemetry.addData("flywheel rpm",data.wheelRPM);
            RobotInitializers.Dashtelemetry.addData("turretOffset",data.YawOffset);

            RobotInitializers.Dashtelemetry.addData("currentRPM", Shooter.FlyWheelEncoder.getRPM());
            Localizer.Update();
        }
    }
}
