package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Storage;

@Config
@TeleOp
public class TuneSpindex extends LinearOpMode {

    public static PIDCoefficients coefs = new PIDCoefficients(0,0,0);


    public static double pos = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);

        Intake intake = new Intake();
        Storage storage = new Storage(intake);
        storage.storagecontroller.setTargetPosition(0);

        waitForStart();

        while(opModeIsActive()){
            RobotInitializers.clearCache();

            Storage.StorageMotor.setPower(storage.storagecontroller.calculatePower(storage.SpindexError()));

            storage.storagecontroller.setPidCoefficients(coefs);

            storage.target = pos;

            RobotInitializers.Dashtelemetry.addData("currpos",storage.FromVtoPi());
        }
    }
}
