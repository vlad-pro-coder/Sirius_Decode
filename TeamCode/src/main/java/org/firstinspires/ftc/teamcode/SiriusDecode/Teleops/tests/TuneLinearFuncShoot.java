package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Shooter;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.ShooterCalculator;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Storage;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Turret;

@TeleOp
@Config
public class TuneLinearFuncShoot extends LinearOpMode {

    public static double rpm = 0;
    public static double hood = 60;

    public static boolean shoot = false;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotInitializers.InitializeFull(hardwareMap);

        Shooter shooter = new Shooter();
        Intake intake = new Intake();
        Storage storage = new Storage(intake);


        while(opModeInInit()){}

        waitForStart();

        while(opModeIsActive()){
            RobotInitializers.clearCache();
            Localizer.Update();

            shooter.SetTargetRPM(rpm);
            shooter.updateFlyWheelSpeed();
            Shooter.PitchController1.setAngle(shooter.FormulaCalculator.NeededServoPosForPitchGiven(hood));

            storage.update();

            if(shoot){
                shoot = false;
                storage.CurrentState = Storage.StorageStates.DOSHOOTACTIONS;
            }

            RobotInitializers.Dashtelemetry.addData("distance", Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(), Turret.goalPosition));
        }

    }
}
