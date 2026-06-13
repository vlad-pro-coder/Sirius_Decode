package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.CRServoPlus;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.ServoPlus;

@TeleOp
@Config
public class TestCrServo extends LinearOpMode {

    public static double power;
    public static int port;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);



        while(opModeInInit()){
            RobotInitializers.clearCache();

            CRServoPlus crServoPlus = new CRServoPlus(RobotInitializers.MotorHub,port, ServoPlus.Direction.FORWARD);

            crServoPlus.setPower(power);
        }


    }
}
