package org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.SiriusDecode.MathHelpers.LinearFunction;


@Config
public class ShooterCalculator {


    public static double DistanceToCancelCompensation = 0;
    public static double FlywheelDiameter = 76.2;

    public static double PosMaxPitch = 355, LowestPitch =
            90 - 20.636;
    public static double PosMinPitch = 30, HighestPitch = 90 - 53.652;


    public double NeededServoPosForPitchGiven(double pitch){
        return LinearFunction.getOutput(PosMaxPitch,PosMinPitch,LowestPitch,HighestPitch,pitch);
    }

    public double givePitch(double Angle){
        Angle = Math.max(LowestPitch,Math.min(Angle,HighestPitch));
        return NeededServoPosForPitchGiven(Angle);
    }

    public double FromBallVelocityToRPM(double velocity){
        return velocity * 60 / (Math.PI * FlywheelDiameter);
    }

    public double FromRPMToBallVelocity(double rpm){
        return rpm * Math.PI * FlywheelDiameter / 60.0;
    }

    public static double TurretOffsetMultiplier = -0.4;
    public static double CompensatedNewRPM = -0.2;
    public static double CompensatorNewHood = 0.00008;

    public static double startangleclose = 51;
    public static double endangleclose = 40;


    public static double startRPMclose = 1850;
    public static double endRPMclose = 2250;

    public static double startDistclose = 1078;
    public static double endDistclose = 2075;


    public static double startanglefar = 41;
    public static double endanglefar = 40;


    public static double startRPMfar = 2920;
    public static double endRPMfar = 3200;

    public static double startDistfar = 3094;
    public static double endDistfar = 3863;


    public static double RPMErrorHoodCompensationFar = 0.012;
    public static double RPMErrorHoodCompensationClose = 0.01;

    public double CompensateRPMSagWitchHood(double ErrorRPM,double currHood,double modifier){
        return currHood + ErrorRPM * modifier;
    }

    public double GetPrequisiteRPM(double dist){
        if(dist > 2500)
            return LinearFunction.getOutput(startRPMfar,endRPMfar,startDistfar,endDistfar,dist);
        return LinearFunction.getOutput(startRPMclose,endRPMclose,startDistclose,endDistclose,dist);

        //return 0.00000599297 * Math.pow(dist,2) + 0.417384 * dist + 1624.68623;
    }

    public double GetPrequisiteHood(double dist){
        if(dist > 2500)
            return LinearFunction.getOutput(startanglefar,endanglefar,startDistfar,endDistfar,dist);
        return LinearFunction.getOutput(startangleclose,endangleclose,startDistclose,endDistclose,dist);

        //return 0.00000495111 * Math.pow(dist,2) - 0.0315278 * dist + 85.66293;
    }





    public static class ShootData{
        public double hoodAngleDegrees,YawOffset,wheelRPM;
        public ShootData(double hoodAngleDegrees,double YawOffset,double wheelRPM){
            this.hoodAngleDegrees = hoodAngleDegrees;
            this.YawOffset = YawOffset;
            this.wheelRPM = wheelRPM;
        }
    }

    public static double OffsetTurretAngle;

    public void OffsetTurret(double rpm, double hood){

        double v_ball = FromRPMToBallVelocity(rpm);

        Localizer.VectorDescriptor vectorMotion = Localizer.GetVelocityVectorOfRobot();

        double dx = Turret.goalPosition.x - Localizer.getCurrentPosition().x;
        double dy = Turret.goalPosition.y - Localizer.getCurrentPosition().y;

        double line = Math.atan2(dy, dx);

        double delta = vectorMotion.orientation - line;

        double towardsComponent = - Math.cos(delta) * vectorMotion.value;
        double sidewaysComponent = Math.sin(delta) * vectorMotion.value;

        RobotInitializers.Dashtelemetry.addData("towardsComponent",towardsComponent);
        RobotInitializers.Dashtelemetry.addData("sidewaysComponent",sidewaysComponent);

        double vballcompensatedX = v_ball * Math.cos(hood) + towardsComponent;


        double OffsetTurret = Math.atan( sidewaysComponent / vballcompensatedX);

        OffsetTurretAngle = Math.toDegrees(TurretOffsetMultiplier * OffsetTurret);
    }



    public ShootData CalculateShootData(double ErrorRPM){
        double distanceToGoal = Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(),Turret.goalPosition);

        double hood_angle = Math.toRadians(GetPrequisiteHood(distanceToGoal));
        double flywheelRPM = GetPrequisiteRPM(distanceToGoal);

        if(distanceToGoal > DistanceToCancelCompensation){
            double newHoodWhileCompensating;
            if(distanceToGoal > 2500)
                newHoodWhileCompensating =  CompensateRPMSagWitchHood(ErrorRPM,Math.toDegrees(hood_angle),RPMErrorHoodCompensationFar);
            else
                newHoodWhileCompensating =  CompensateRPMSagWitchHood(ErrorRPM,Math.toDegrees(hood_angle),RPMErrorHoodCompensationClose);

            RobotInitializers.Dashtelemetry.addData("hood angle",newHoodWhileCompensating);
            RobotInitializers.Dashtelemetry.addData("distance", distanceToGoal);
            RobotInitializers.Dashtelemetry.addData("targetRPM", flywheelRPM);

            OffsetTurret(flywheelRPM,Math.max(HighestPitch,Math.min(newHoodWhileCompensating,LowestPitch)));

            return new ShootData(Math.max(HighestPitch,Math.min(newHoodWhileCompensating,LowestPitch)),0,flywheelRPM);
        }

        double v_ball = FromRPMToBallVelocity(flywheelRPM);

        RobotInitializers.Dashtelemetry.addData("flywheel normal",FromBallVelocityToRPM(v_ball));
        RobotInitializers.Dashtelemetry.addData("hood_angle",Math.toDegrees(hood_angle));

        Localizer.VectorDescriptor vectorMotion = Localizer.GetVelocityVectorOfRobot();

        double dx = Turret.goalPosition.x - Localizer.getCurrentPosition().x;
        double dy = Turret.goalPosition.y - Localizer.getCurrentPosition().y;

// Angle from robot to goal
        double line = Math.atan2(dy, dx);

        double delta = vectorMotion.orientation - line;

        double towardsComponent = - Math.cos(delta) * vectorMotion.value;
        double sidewaysComponent = Math.sin(delta) * vectorMotion.value;

        RobotInitializers.Dashtelemetry.addData("towardsComponent",towardsComponent);
        RobotInitializers.Dashtelemetry.addData("sidewaysComponent",sidewaysComponent);

        double vballcompensatedX = v_ball * Math.cos(hood_angle) + towardsComponent;

        double newHood = hood_angle + -towardsComponent * CompensatorNewHood;
        double newRPM = flywheelRPM + -towardsComponent * CompensatedNewRPM;

        //double hoodCompensated = CompensateRPMSagWitchHood(ErrorRPM,Math.toDegrees(newHood),RPMErrorHoodCompensationWhileMoving);

        /*double vballY = v_ball * Math.sin(hood_angle);

        RobotInitializers.Dashtelemetry.addData("vballY = ", vballY);
        RobotInitializers.Dashtelemetry.addData("vballcompensatedX = ", vballcompensatedX);

        double timeFlight = distanceToGoal / (v_ball * Math.cos(hood_angle));

        double hood_new = Math.atan( vballY / vballcompensatedX);

        double VxBallElimCompensation = distanceToGoal / timeFlight + towardsComponent * ElimCompensatiorMultiplier;
        double VyBall = v_ball * Math.sin(hood_new);

        double NeededSlowedBall = Math.sqrt(VxBallElimCompensation * VxBallElimCompensation + VyBall * VyBall);
        double newRPM = FromBallVelocityToRPM(NeededSlowedBall);*/

        double OffsetTurret = Math.atan( sidewaysComponent / vballcompensatedX);

        RobotInitializers.Dashtelemetry.addData("Difference in RPM = ", newRPM - flywheelRPM);
        RobotInitializers.Dashtelemetry.addData("Difference in degrees hood", Math.toDegrees(newHood - hood_angle));


        return new ShootData(hood_angle,Math.toDegrees(TurretOffsetMultiplier * OffsetTurret),flywheelRPM);
    }
}
