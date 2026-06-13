package org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.CRServoPlus;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.CachedMotor;
import org.firstinspires.ftc.teamcode.SiriusDecode.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.ServoPlus;

import java.util.Arrays;
import java.util.List;

@Config
public class Shooter {
    public static CRServoPlus Launcher1,Launcher2;

    public static CachedMotor FlyWheelEncoder;

    public static ServoPlus PitchController1;

    public PIDController LauncherSpeedController = new PIDController(0.0004,0,0);

    public ShooterCalculator FormulaCalculator;

    public Shooter(){
        FormulaCalculator = new ShooterCalculator();
    }


    public void setPowerEquilibrated(double power){
        double p = 12.8 / RobotInitializers.VOLTAGE;
        //RobotInitializers.Dashtelemetry.addLine("equilibrated power" + power * p);
        Launcher1.setPower(power * p);
        Launcher2.setPower(power * p);
    }


    public static double Fs = 0.08;
    public static double Kv = 0.00027;
    public static double Ka = 0;

    public void updateFlyWheelSpeed(){
        if(LauncherSpeedController.getTargetPosition() < 1000)
        {
            Launcher1.setPower(Fs);
            Launcher2.setPower(Fs);
            return;
        }
        //RobotInitializers.Dashtelemetry.addData("rpm shooter",FlyWheelEncoder.getRPM());
        double LaunchSpeed = LauncherSpeedController.calculatePower(FlyWheelEncoder.getRPM());
        LaunchSpeed += Kv * LauncherSpeedController.getTargetPosition() + Math.signum(LauncherSpeedController.getTargetPosition() - FlyWheelEncoder.getRPM()) * Fs;
        setPowerEquilibrated(LaunchSpeed);
    }

    public void UpdateTurretAndPitch(){



        ShooterCalculator.ShootData data = FormulaCalculator.CalculateShootData(RPMError());

        SetTargetRPM(data.wheelRPM);

        PitchController1.setAngle(FormulaCalculator.NeededServoPosForPitchGiven(data.hoodAngleDegrees));
    }

    public void SetAngleToHood(double hood){
        PitchController1.setAngle(FormulaCalculator.NeededServoPosForPitchGiven(hood));
    }




    public void SetTargetRPM(double targetRPM){
        LauncherSpeedController.setTargetPosition(targetRPM);
    }

    public boolean RPMError(double RPMerror){
        return Math.abs(LauncherSpeedController.getTargetPosition() - FlyWheelEncoder.getRPM()) < RPMerror;
    }

    public double RPMError(){
        return LauncherSpeedController.getTargetPosition() - FlyWheelEncoder.getRPM();
    }


}

