package org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.CRServoPlus;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.CachedMotor;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.MonoChromeGlobalShutterCamera;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.RGBsensor;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.ServoPlus;

import java.util.List;

@Config
public class RobotInitializers {
    public static List<LynxModule> hubs;

    public static Telemetry Dashtelemetry;
    public static IMU imu;
    public static DcMotorControllerEx ControlHubMotors, ExpansionHubMotors;
    public static ServoControllerEx ControlHubServos, ExpansionHubServos, ServoHub,MotorHub;
    public static DigitalChannelController ControlHubDigital,ExpansionHubDigital;

    public static AnalogInputController ControlHubAnalog,ExpansionHubAnalog;
    public static MonoChromeGlobalShutterCamera camera;
    public static double VOLTAGE = 12;
    public static boolean isDisabled(){
        return !hubs.get(0).isEngaged();
    }
    public static void disable(){
        for(LynxModule l : hubs){
            l.disengage();
        }
    }
    public static void enable(){
        for(LynxModule l : hubs){
            l.engage();
        }
    }
    public static void  enableDashTelemetry(){
        Dashtelemetry = FtcDashboard.getInstance().getTelemetry();
    }
    public static void InitializeHubs(HardwareMap hm){
        InitializeHubs(hm, false);
    }

    public static void InitializeHubs(HardwareMap hm, boolean b) {
        enableDashTelemetry();
        if(hubs != null) return;
        hubs = hm.getAll(LynxModule.class);
//        if(b)
//            disable();

        ControlHubMotors = hm.get(DcMotorControllerEx.class, "Control Hub");
        //ExpansionHubMotors = hm.get(DcMotorControllerEx.class, "Expansion Hub 2");
        RobotLog.i("Motors initialized");
        ControlHubServos = hm.get(ServoControllerEx.class, "Control Hub");
        //ExpansionHubServos = hm.get(ServoController.class, "Expansion Hub 2");
        RobotLog.i("Servos (chub ehub) initialized");
        ControlHubDigital = hm.get(DigitalChannelController.class,"Control Hub");
        //ExpansionHubDigital = hm.get(DigitalChannelController.class,"Expansion Hub 2");
        RobotLog.i("Digital initialized");
        ControlHubAnalog = hm.get(AnalogInputController.class,"Control Hub");
        //ExpansionHubAnalog = hm.get(AnalogInputController.class,"Expansion Hub 2");
        RobotLog.i("Analog initialized");

        ServoHub = hm.get(ServoControllerEx.class, "Servo Hub 3");
        MotorHub = hm.get(ServoControllerEx.class, "Servo Hub 4");

        //IMUBNO085.controller = hm.get(DigitalChannelController.class, "Expansion Hub 1")

        MotorConfigurationType mct;

        for(int i = 0; i < 4; i++){
            mct = ControlHubMotors.getMotorType(i);
            mct.setAchieveableMaxRPMFraction(1);

            ControlHubMotors.setMotorType(i, mct);

            /*mct = ExpansionHubMotors.getMotorType(i);
            mct.setAchieveableMaxRPMFraction(1);

            ExpansionHubMotors.setMotorType(i, mct);*/



        }

//        imu = hm.get(IMUBNO085.class, "extIMU");
        imu = hm.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        )));
//        Orientation o = new Orientation(
//                AxesReference.EXTRINSIC,
//                AxesOrder.ZXY,
//                AngleUnit.DEGREES,
//                0, 0, 0, 0
//        );
//        Robot.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(o)));

        imu.resetYaw();

        VOLTAGE = hm.getAll(VoltageSensor.class).get(0).getVoltage();

        for(LynxModule l : hubs){
            l.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public static void clearCache(){
        clearCache(true);
    }
    public static long loopTime = 0;
    private static ElapsedTime logFreq = new ElapsedTime();
    public static void clearCache(boolean update){
        for(LynxModule l : hubs){
            l.clearBulkCache();
        }
        if(update) {
            Dashtelemetry.update();
            Dashtelemetry.addData("frequency", String.valueOf(1000.f / (System.currentTimeMillis() - loopTime)));
        }
        if(logFreq.seconds() >= 1) {
            RobotLog.ii("frequency", String.valueOf(1000.f / (System.currentTimeMillis() - loopTime)));
            logFreq.reset();
        }
        loopTime = System.currentTimeMillis();
    }
    public static void InitializeFull(HardwareMap hm){
        InitializeHubs(hm, true);
        InitializeChassis();
        InitializeLocalizer(hm);
        InitializeIntake();
        InitializeStorage(hm);
        InitializeShooter();
        InitializeTurret();
        //InitializeCamera(hm);
    }

    public static void InitializeCamera(HardwareMap hm){
        camera = hm.get(MonoChromeGlobalShutterCamera.class,"camera");
    }


    public static void InitializeChassis(){
        Chassis.FL = new CachedMotor(ControlHubMotors, 1, DcMotorSimple.Direction.REVERSE);
        Chassis.FR = new CachedMotor(ControlHubMotors, 3, DcMotorSimple.Direction.FORWARD);
        Chassis.BL = new CachedMotor(ControlHubMotors, 0, DcMotorSimple.Direction.REVERSE);
        Chassis.BR = new CachedMotor(ControlHubMotors, 2, DcMotorSimple.Direction.FORWARD);

        Chassis.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Chassis.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Chassis.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Chassis.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void InitializeLocalizer(HardwareMap hm){
        Localizer.Initialize(hm);
        //Odo.init(hm);
    }

    public static void InitializeStorage(HardwareMap hm){
        Storage.StorageMotor = new CRServoPlus(ServoHub,1, ServoPlus.Direction.REVERSE);
        Storage.colorsensor = hm.get(RGBsensor.class,"colorsensor");
        Storage.Putulica = new ServoPlus(MotorHub,0,Servo.Direction.FORWARD);//305 engaged 210 dis
        Storage.AbsoluteEncoder = new AnalogInput(ControlHubAnalog,0);
        Storage.sensor = new DigitalChannelImpl(ControlHubDigital,6);
    }

    public static void InitializeTurret(){
        Turret.TurretRawPosition = new AnalogInput(ControlHubAnalog,2);
        //Turret.mk21 = new ServoPlus(MotorHub,2,Servo.Direction.FORWARD);
        //Turret.mk22 = new ServoPlus(MotorHub,3,Servo.Direction.FORWARD);
        Turret.mk21 = new ServoImplEx(MotorHub,2, ServoConfigurationType.getStandardServoType());
        Turret.mk22 = new ServoImplEx(MotorHub,3,ServoConfigurationType.getStandardServoType());

        //Turret.cmk21 = new CRServoPlus(MotorHub,2, ServoPlus.Direction.FORWARD);
        //Turret.cmk22 = new CRServoPlus(MotorHub,3, ServoPlus.Direction.FORWARD);

        Turret.EncoderBore = new CachedMotor(ControlHubMotors,3,DcMotorSimple.Direction.FORWARD);
        Turret.EncoderBore.ActivateEncoder(8192);
    }

    public static void InitializeShooter(){
        Shooter.PitchController1 = new ServoPlus(MotorHub,1, Servo.Direction.FORWARD); //60 - 340

        Shooter.Launcher1 = new CRServoPlus(MotorHub,4, ServoPlus.Direction.FORWARD);
        Shooter.Launcher2 = new CRServoPlus(MotorHub,5,ServoPlus.Direction.FORWARD);

        Shooter.FlyWheelEncoder = new CachedMotor(ControlHubMotors,0,DcMotorSimple.Direction.FORWARD);
        Shooter.FlyWheelEncoder.ActivateEncoder(28);
    }

    public static void InitializeIntake()
    {
        Intake.spinner = new CRServoPlus(ServoHub,0,ServoPlus.Direction.REVERSE);
    }

    /*public static void InitializePTO(){
        PTO1 = new ServoPlus(ServoHub,0, Servo.Direction.FORWARD); // dis 320
        PTO1 = new ServoPlus(ServoHub,2, Servo.Direction.FORWARD); // dis 320
    }*/

}