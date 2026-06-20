package org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.SiriusDecode.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.SiriusDecode.Pathing.MotionPredictivControl;
import org.firstinspires.ftc.teamcode.SiriusDecode.Pathing.PurePersuit;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.CachedMotor;

@Config
public class Chassis {

    public static CachedMotor FL, FR, BL, BR;
    public static PurePersuit PurePersuitTrajectory;
    public static ElapsedTime timeafterMPC = new ElapsedTime();
    public static double lasttime = 0;

    public static double OffsetFromHoldingHeading = 0;
    public static SparkFunOTOS.Pose2D velocities = new SparkFunOTOS.Pose2D(0,0,0);

    public static double speedModifierX = 1,speedModifierY = 1,speedModifierH = 1;
    public static MotionPredictivControl MotionPredictivControlTrajectory = new MotionPredictivControl(10);
    public enum trajectoryStates{
        FREEWILL,
        FOLLOWINGPUREPERSUIT,
        FOLLOWINGMOTIONPREDICTIVECONTROL,
        OFF,
        USENOTHING,
        UPDATEHEADINGONLY
    }
    public static trajectoryStates usedTrajectory = trajectoryStates.FREEWILL;
    public static SparkFunOTOS.Pose2D target = new SparkFunOTOS.Pose2D(0,0,0);
/*    public static PIDController Strafe = new PIDController(-0.0055,0,-0.0004),
            Forward = new PIDController(0.003,0,0.0003),
            Heading = new PIDController(0.6,0,0.02);*/

    public static PIDController Strafe = new PIDController(0.01,0,0.0015),
            Forward = new PIDController(0.01,0,0.0015),
            Heading = new PIDController(-1.7,0,-0.1);



    public static PIDCoefficients HeadingVibrate = new PIDCoefficients(-1.7,0,-3)
            ,HeadingNormal = new PIDCoefficients(-1.7,0,-0.1)
    ;


    public static double Forwardkv = 0,Strafekv = 0,Headingkv = 0;
    public static double Forwardka = 0,Strafeka = 0,Headingka = 0;

    static{
        Strafe.setFreq(30);
        Forward.setFreq(30);
        Heading.setFreq(30);

        Strafe.kS = -0.0;
        Forward.kS = 0.00;
        Heading.kS = -0.0; //-0.035

    }

    public static void drive(double x, double y, double r){
        double d = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);
        double fl, bl, fr, br;

        fl = (y + x + r) / d;
        bl = (y - x + r) / d;
        fr = (y - x - r) / d;
        br = (y + x - r) / d;

        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);
    }

    public static void driveFieldCentricWhileHoldingHeading(double x, double y,double r,double targetH,double OffsetAngle){
        double botHeading = Localizer.getCurrentPosition().h + OffsetAngle;
//        if(TeleopsStarter.team == TEAM.RED)
        botHeading += Math.PI;

        OffsetFromHoldingHeading += r;

        double Herror = Localizer.normalizeRadians(targetH + Math.toRadians(OffsetFromHoldingHeading) - Localizer.getCurrentPosition().h);
        double hP = Heading.calculatePower(Herror);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(hP), 1);
        double frontLeftPower = (rotY + rotX + hP) / denominator;
        double backLeftPower = (rotY - rotX + hP) / denominator;
        double frontRightPower = (rotY - rotX - hP) / denominator;
        double backRightPower = (rotY + rotX - hP) / denominator;

        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);
        FR.setPower(frontRightPower);
        BR.setPower(backRightPower);
    }

    public static void driveFieldCentric(double x, double y, double r,double OffsetAngle){



        double botHeading = Localizer.getCurrentPosition().h + OffsetAngle;
//        if(TeleopsStarter.team == TEAM.RED)
            botHeading += Math.PI;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(r), 1);
        double frontLeftPower = (rotY + rotX + r) / denominator;
        double backLeftPower = (rotY - rotX + r) / denominator;
        double frontRightPower = (rotY - rotX - r) / denominator;
        double backRightPower = (rotY + rotX - r) / denominator;

        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);
        FR.setPower(frontRightPower);
        BR.setPower(backRightPower);
    }

    public static void setTargetPosition(SparkFunOTOS.Pose2D pos){

        switch (usedTrajectory){
            case FOLLOWINGMOTIONPREDICTIVECONTROL:
                MotionPredictivControlTrajectory.setGoalPos(pos);
        }

        pos.h = Localizer.normalizeRadians(pos.h);
        Strafe.setTargetPosition(0);
        Forward.setTargetPosition(0);
        Heading.setTargetPosition(0);
        target = pos;
    }

    public static SparkFunOTOS.Pose2D getTargetPosition(){
        return target;
    }
    public static void setHeading(double heading){
        setTargetPosition(new SparkFunOTOS.Pose2D(target.x, target.y, heading));
    }

    public static boolean IsPositionDone(double error_distance){
        switch (usedTrajectory){
            case FREEWILL:
                return Localizer.getDistanceFromTwoPoints(getTargetPosition(),Localizer.getCurrentPosition()) <= error_distance;
            case FOLLOWINGPUREPERSUIT:
                return PurePersuitTrajectory.TrajectoryDone(error_distance);
            case FOLLOWINGMOTIONPREDICTIVECONTROL:
                //maybe in the future
        }
        return false;
    }
    public static boolean IsHeadingDone(double error_heading){
        switch (usedTrajectory){
            case FREEWILL:
                return Localizer.getAngleDifference(getTargetPosition().h,Localizer.getCurrentPosition().h) <= Math.toRadians(error_heading);
            case FOLLOWINGPUREPERSUIT:
                return PurePersuitTrajectory.AngleDone(error_heading);
            case FOLLOWINGMOTIONPREDICTIVECONTROL:
                //maybe in the future
        }
       return false;
    }


    public static void update(){
        double headingRespectCoef = 1;

        double importanceOfX = 1;
        double importanceOfY = 1;
        switch (usedTrajectory){
            case FOLLOWINGPUREPERSUIT:
                RobotLog.ii("current purepersuit angle",String.valueOf(PurePersuitTrajectory.FromLinesGeneratePointToFollow()));
                setTargetPosition(PurePersuitTrajectory.FromLinesGeneratePointToFollow());
                headingRespectCoef = PurePersuitTrajectory.GetHeadingRespectCoef();
                break;
            case FOLLOWINGMOTIONPREDICTIVECONTROL:
                if(timeafterMPC.seconds() > MotionPredictivControl.delta_t) {
                    velocities = MotionPredictivControlTrajectory.GetTargetVelocities();
                }
                double timepasssed = timeafterMPC.seconds() - lasttime;
                double h = Localizer.normalizeRadians(Localizer.getCurrentPosition().h);
                double velocityForward = (Math.cos(h) * velocities.x + Math.sin(h) * velocities.y);
                double velocityStrafe = (Math.sin(h) * velocities.x - Math.cos(h) * velocities.y);

                double accelerationForward = (velocityForward - Localizer.getVelocity().y) / MotionPredictivControl.delta_t;
                double accelerationStrafe = (velocityStrafe - Localizer.getVelocity().x) / MotionPredictivControl.delta_t;
                double accelerationHeading = (velocities.h - Localizer.getVelocity().h) / MotionPredictivControl.delta_t;

                double ForwardPower = velocityForward * Forwardkv + accelerationForward * Forwardka;
                double StrafePower = velocityStrafe * Strafekv + accelerationStrafe * Strafeka;
                double HeadingPower = velocities.h * Headingkv + accelerationHeading * Headingka;
                drive(StrafePower, ForwardPower, HeadingPower);
                lasttime = timeafterMPC.seconds();
                return ;
            case OFF:
                drive(0,0,0);
                return;
            case USENOTHING:
                return;
        }

        SparkFunOTOS.Pose2D normal = new SparkFunOTOS.Pose2D(
                getTargetPosition().x - Localizer.getCurrentPosition().x,
                getTargetPosition().y - Localizer.getCurrentPosition().y,
                getTargetPosition().h - Localizer.getCurrentPosition().h);

        double Herror = Localizer.normalizeRadians(normal.h);
        double h = Localizer.getCurrentPosition().h;
        while(h < 0) h += Math.PI * 2;
        while(h > 2*Math.PI) h -= Math.PI * 2;
        SparkFunOTOS.Pose2D error = new SparkFunOTOS.Pose2D(
                Math.cos(h) * normal.x + Math.sin(h) * normal.y,
                Math.sin(h) * normal.x - Math.cos(h) * normal.y,
                Herror
        );

        double yP = Forward.calculatePower(error.x);
        double xP = Strafe.calculatePower(error.y);
        double hP = Heading.calculatePower(error.h);

        /*RobotInitializers.Dashtelemetry.addData("xError", error.x);
        RobotInitializers.Dashtelemetry.addData("yError", error.y);
        RobotInitializers.Dashtelemetry.addData("hError", Math.toDegrees(error.h));*/
        double p = 1;
        /*if(RobotInitializers.VOLTAGE > 12.8){
            p *= 12.8 / RobotInitializers.VOLTAGE;
        }*/
        p *= (slowFollow ? 0.6 : 0.93);


        //RobotInitializers.Dashtelemetry.addData("x power", yP * p);
        //RobotInitializers.Dashtelemetry.addData("y power", -xP * p);
        //RobotInitializers.Dashtelemetry.addData("h power", hP * p * headingRespectCoef);

        if(usedTrajectory == trajectoryStates.FOLLOWINGPUREPERSUIT && headingRespectCoef != 1){
            double MAX_ANGLE = Math.toRadians(180.0);

            double headingError = Math.abs(
                    Localizer.getAngleDifference(
                            getTargetPosition().h,
                            Localizer.getCurrentPosition().h
                    )
            );

            //RobotInitializers.Dashtelemetry.addData("headingError",headingError);

            double procentile = 1 - headingError / MAX_ANGLE;
            procentile = Math.min(1.0, procentile);

            //RobotInitializers.Dashtelemetry.addData("procentile",procentile);

            double translationScale = 1.0 - headingRespectCoef;

            importanceOfX = procentile * translationScale;
            importanceOfY = procentile * translationScale;
        }

        //obotInitializers.Dashtelemetry.addData("impotanceOfX",importanceOfX);
        //RobotInitializers.Dashtelemetry.addData("impotanceOfY",importanceOfY);

        if(slowFollow && yP > p)
            yP = p;
        if(slowFollow && xP > p)
            xP = p;


        drive(yP * importanceOfX, -xP * importanceOfY, hP * p);
    }
    private static boolean slowFollow = false;
    public static void setSlowFollow(boolean set){
        slowFollow = set;
    }
}
