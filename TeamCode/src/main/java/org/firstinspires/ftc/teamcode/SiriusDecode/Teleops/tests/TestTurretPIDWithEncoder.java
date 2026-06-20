package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Turret;

@TeleOp
@Config
public class TestTurretPIDWithEncoder extends LinearOpMode {


    public enum Mods{
        SetPower,
        FollowGoal
    }
    public static Mods hub = Mods.SetPower;
    public static double power = 0;
    public static boolean reset = false;

    public static PIDCoefficients coefs = new PIDCoefficients(0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {

        RobotInitializers.InitializeFull(hardwareMap);

        Turret turret = new Turret();


        waitForStart();

        while(opModeIsActive()){

            Localizer.Update();
            RobotInitializers.clearCache();

            //turret.turretController.setPidCoefficients(coefs);

            if(reset){
                Turret.EncoderBore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Turret.EncoderBore.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                reset = false;
            }

            if(hub == Mods.SetPower)
                turret.SetTurretPower(power);
            else if(hub == Mods.FollowGoal)
                turret.updateTurretEncoder(Localizer.getCurrentPosition());
        }
    }
}
