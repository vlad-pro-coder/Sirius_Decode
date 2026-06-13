package org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RollingAverage;

import org.firstinspires.ftc.teamcode.SiriusDecode.MathHelpers.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.CachedMotor;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.ServoPlus;


@Config
public class Turret {

    public static double AngleOffsetFrom0 = 130;
    public static double BreathRoomPos = 0.02;
    public static double OppositionPos = 0.01;
    public static double CompensateMechanicPlayCoef = 0;

    public static double RotationalVelocityTurretCompensator = -0.09;
    public static SparkFunOTOS.Pose2D goalPosition = new SparkFunOTOS.Pose2D(1470,3110,0);

    public static double pas = 10;

    public double currpos = 180;

    public static double MaxAngle = 355;

    public static ServoImplEx mk21,mk22;
    public static AnalogInput TurretRawPosition;
    public static CachedMotor EncoderBore;

    public Turret(){

        mk21.setPwmRange(new PwmControl.PwmRange(500 , 2500));
        mk22.setPwmRange(new PwmControl.PwmRange(500 , 2500));

        mk21.setDirection(Servo.Direction.FORWARD);
        mk22.setDirection(Servo.Direction.FORWARD);
    }
    public static double offsetTurretX = -1.517;
    public static double offsetTurretY = -20.437;

    public static double offsetH = 0;
    public static double gearRatio   = 120.0/30.0;

    private double fromEncoderToRads() {
        double fromDegToTicks = (AngleOffsetFrom0 / 360.0) * EncoderBore.getEncoderResolution();
        double ticks =  - EncoderBore.getCurrentPosition() - fromDegToTicks;

        //RobotInitializers.Dashtelemetry.addData("true ticks", ticks);
        double ticksPerRev = EncoderBore.getEncoderResolution() * gearRatio;
        return ticks * 2.0 * Math.PI / ticksPerRev;
    }

    private double LiniarizedTargetAngle(double targetAngle,double currContinuosAngle){
        double bestdifference = 1e9, bestcontinuosAngle = 1e9;

        double distNormalizedContinuos = Localizer.cwDistance(2*Math.PI - Math.toRadians(AngleOffsetFrom0),targetAngle);

        //RobotInitializers.Dashtelemetry.addData("clockwise distance",distNormalizedContinuos);

        while(distNormalizedContinuos<=Math.toRadians(MaxAngle))
        {
            if(bestdifference > Math.abs(distNormalizedContinuos - currContinuosAngle))
            {
                bestdifference = Math.abs(distNormalizedContinuos - currContinuosAngle);
                bestcontinuosAngle = distNormalizedContinuos;
            }
            distNormalizedContinuos+=2*Math.PI;
        }
        if(bestcontinuosAngle == 1e9)
            return 1e9;
        return currContinuosAngle - bestcontinuosAngle;
    }

    public SparkFunOTOS.Pose2D TurretFieldRelativePosition(SparkFunOTOS.Pose2D robotpose){

        double trueOutputBallx = robotpose.x + offsetTurretX * Math.cos(robotpose.h + offsetH) - offsetTurretY * Math.sin(robotpose.h + offsetH);
        double trueOutputBally = robotpose.y + offsetTurretX * Math.sin(robotpose.h + offsetH) + offsetTurretY * Math.cos(robotpose.h + offsetH);

        return new SparkFunOTOS.Pose2D(trueOutputBallx,trueOutputBally, robotpose.h);
    }

    public void updateTurret(SparkFunOTOS.Pose2D robotPose){

        robotPose = TurretFieldRelativePosition(robotPose);
        //RobotInitializers.Dashtelemetry.addData("pos turret",robotPose);
        //RobotInitializers.Dashtelemetry.addData("OffsetFromMoving", ShooterCalculator.OffsetTurretAngle);

        //RobotInitializers.Dashtelemetry.addData("turretpos", TurretRawPosition.getVoltage() / TurretRawPosition.getMaxVoltage() * MaxAngle);
        double robotHeading = robotPose.h;

        double dx = goalPosition.x - robotPose.x;
        double dy = goalPosition.y - robotPose.y;

        double targetGlobalHeading = Math.atan2(dy, dx);

        //RobotInitializers.Dashtelemetry.addData("targetGlobalHeading",targetGlobalHeading);


        double ShouldHaveTurretHeading = Math.toDegrees(Localizer.normalizeTo0To2PI((targetGlobalHeading - robotHeading + Math.toRadians(AngleOffsetFrom0) + Math.toRadians(ShooterCalculator.OffsetTurretAngle) + RotationalVelocityTurretCompensator * Localizer.getVelocity().h))); //- Math.toRadians(data.YawOffset);
        double CompensateMechanicalPlay = Math.max(Math.min(LiniarizedTargetAngle(ShouldHaveTurretHeading,fromEncoderToRads()) * CompensateMechanicPlayCoef,10),-10);
        RobotInitializers.Dashtelemetry.addData("OffsetFromMoving", ShooterCalculator.OffsetTurretAngle);
        //RobotInitializers.Dashtelemetry.addData("ShouldHaveTurretHeading",ShouldHaveTurretHeading);
        ShouldHaveTurretHeading = Math.max(Math.min(ShouldHaveTurretHeading + CompensateMechanicalPlay,MaxAngle),0);

        if(currpos < ShouldHaveTurretHeading)
            currpos = Math.min(currpos + pas,ShouldHaveTurretHeading);
        else if(currpos > ShouldHaveTurretHeading)
            currpos = Math.max(currpos - pas,ShouldHaveTurretHeading);

        currpos = Math.min(Math.max(currpos ,0),MaxAngle);

        setAngle(currpos);

    }

    public void setAngle(double angle){
        //RobotInitializers.Dashtelemetry.addData("angle",angle);
        //RobotInitializers.Dashtelemetry.addData("m21",Math.max(BreathRoomPos,Math.min(1-BreathRoomPos,angle/MaxAngle+OppositionPos)));


        mk21.setPosition(Math.max(BreathRoomPos,Math.min(1-BreathRoomPos,angle/MaxAngle+OppositionPos)));
        mk22.setPosition(Math.max(BreathRoomPos,Math.min(1-BreathRoomPos,angle/MaxAngle-OppositionPos)));
    }

    public void setAngleToInit(double val){
        mk21.setPosition(val);
        mk22.setPosition(val);
    }
}
