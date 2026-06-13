package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;

import static org.firstinspires.ftc.teamcode.SiriusDecode.TeleopsStarter.storage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Shooter;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.ShooterCalculator;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Storage;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Turret;

@TeleOp
@Config
public class TestHoodAngles extends LinearOpMode {

    public static double targetRPM = 0;

    public static boolean transfer = true;

    public static double targetPitch = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);

        Shooter shooter = new Shooter();
        Intake intake = new Intake();
        Storage storage = new Storage(intake);
        Turret turret = new Turret();

        while(opModeInInit()){
            RobotInitializers.clearCache();
            Localizer.Update();
        }

        waitForStart();

        while(opModeIsActive()){
            RobotInitializers.clearCache();

            turret.updateTurret(Localizer.getCurrentPosition());
            shooter.SetTargetRPM(targetRPM);
            shooter.updateFlyWheelSpeed();

            if(transfer){
                storage.CurrentState = Storage.StorageStates.DOSHOOTACTIONS;
                transfer = false;
            }

            RobotInitializers.Dashtelemetry.addData("distance",Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(), Turret.goalPosition));
            RobotInitializers.Dashtelemetry.addData("RPM Flywheel",Shooter.FlyWheelEncoder.getRPM());

            Shooter.PitchController1.setAngle(shooter.FormulaCalculator.NeededServoPosForPitchGiven(targetPitch));
            storage.update();
            Localizer.Update();
        }

    }
}
