package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Shooter;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Storage;

@TeleOp
@Config
public class TestStorage extends LinearOpMode {

    public static boolean startLoad = false;


    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);

        Intake intake = new Intake();
        Storage storage = new Storage(intake);
        Shooter shooter = new Shooter();


        waitForStart();

        while (opModeIsActive()){
            RobotInitializers.clearCache();
            intake.takeIn();

            if(startLoad)
            {
                storage.CurrentState = Storage.StorageStates.DOSHOOTACTIONS;
                intake.takeIn();
                shooter.setPowerEquilibrated(0.7);
                startLoad = false;
            }

            storage.update();
            RobotInitializers.Dashtelemetry.update();

        }
    }
}