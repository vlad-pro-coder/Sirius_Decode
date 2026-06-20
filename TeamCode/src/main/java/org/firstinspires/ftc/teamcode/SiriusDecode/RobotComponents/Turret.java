package org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.RollingAverage;

import org.firstinspires.ftc.teamcode.SiriusDecode.MathHelpers.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.SiriusDecode.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.CRServoPlus;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.CachedMotor;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.ServoPlus;


@Config
public class Turret {

    public static double AngleOffsetFrom0 = 119;//130
    public static double BreathRoomPos = 0.05;
    public static double OppositionPos = 0.0;
    public static double CompensateMechanicPlayCoef = 0;

    public static double globalError;

    public static double ManualOffset = 0;

    public static double RotationalVelocityTurretCompensator = -0.09;
    public static SparkFunOTOS.Pose2D goalPosition = new SparkFunOTOS.Pose2D(1470,3110,0);

    public static PIDCoefficients higherrorturretcoefs = new PIDCoefficients(-0.7,0,-0.02);
    public PIDCoefficients lowerrorturretcoefs = new PIDCoefficients(-1.3,0,0);

    public static double MultiplierPID = 1.7;
    public PIDController turretController = new PIDController(new PIDCoefficients(0,0,0));

    public static double pas = 10;

    public double currpos = AngleOffsetFrom0;

    public static double MaxAngle = 355;//355

    public static ServoImplEx mk21,mk22;

    public static CRServoPlus cmk21,cmk22;
    public static AnalogInput TurretRawPosition;
    public static CachedMotor EncoderBore;

    public Turret(){

        mk21.setPwmRange(new PwmControl.PwmRange(500 , 2500));
        mk22.setPwmRange(new PwmControl.PwmRange(500 , 2500));

        mk21.setDirection(Servo.Direction.FORWARD);
        mk22.setDirection(Servo.Direction.FORWARD);

        ManualOffset = 0;
    }
    public static double offsetTurretX = -1.517;
    public static double offsetTurretY = -20.437;

    private final double gearRatio   = 120.0/30.0;

    public SparkFunOTOS.Pose2D TurretFieldRelativePosition(SparkFunOTOS.Pose2D robotpose){

        double cos = Math.cos(robotpose.h);
        double sin = Math.sin(robotpose.h);

        double trueOutputBallx = robotpose.x + offsetTurretX * cos - offsetTurretY * sin;
        double trueOutputBally = robotpose.y + offsetTurretX * sin + offsetTurretY * cos;

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


        double ShouldHaveTurretHeading = targetGlobalHeading - robotHeading + ShooterCalculator.OffsetTurretAngle + RotationalVelocityTurretCompensator * Localizer.getVelocity().h + ManualOffset; //- Math.toRadians(data.YawOffset);

        double ShouldHaveTurretHeadingNormalized = Math.toDegrees(Localizer.normalizeTo0To2PI(ShouldHaveTurretHeading+ Math.toRadians(AngleOffsetFrom0)));
        double CompensateMechanicalPlay = 0;//Math.max(Math.min(Math.toDegrees(LiniarizedTargetAngle(ShouldHaveTurretHeading,fromEncoderToRads())) * CompensateMechanicPlayCoef,10),-10);
        /*RobotInitializers.Dashtelemetry.addData("Compensate play", CompensateMechanicalPlay);
        RobotInitializers.Dashtelemetry.addData("error", Math.toDegrees(LiniarizedTargetAngle(ShouldHaveTurretHeading,fromEncoderToRads())));
        RobotInitializers.Dashtelemetry.addData("OffsetFromMoving", ShooterCalculator.OffsetTurretAngle);
        RobotInitializers.Dashtelemetry.addData("ShouldHaveTurretHeading",ShouldHaveTurretHeading);*/
        //RobotLog.ii("manual offset", String.valueOf(ManualOffset));
        ShouldHaveTurretHeadingNormalized = Math.max(Math.min(ShouldHaveTurretHeadingNormalized + CompensateMechanicalPlay,MaxAngle),0);

        /*if(currpos < ShouldHaveTurretHeadingNormalized)
            currpos = Math.min(currpos + pas,ShouldHaveTurretHeadingNormalized);
        else if(currpos > ShouldHaveTurretHeadingNormalized)
            currpos = Math.max(currpos - pas,ShouldHaveTurretHeadingNormalized);
*/

        setAngle(ShouldHaveTurretHeadingNormalized);

    }

    public void SetTurretPower(double power){
        cmk21.setPower(power);
        cmk22.setPower(power);
    }

    public static double FeedForwardTurret = 0;
    public static double Kvangular = 0.05;

    private final double DegToTick = (AngleOffsetFrom0 / 360.0) * 8192 * gearRatio;

    private final double ticksPerRev = 8192 * gearRatio / (2.0 * Math.PI);

    private double fromEncoderToRads() {
        //RobotInitializers.Dashtelemetry.addData("Offset", DegToTick);
        double ticks =  - EncoderBore.getCurrentPosition() + DegToTick;

        //RobotInitializers.Dashtelemetry.addData("true ticks", ticks);

        //RobotInitializers.Dashtelemetry.addData("ticksPerRev", ticksPerRev);
        return ticks  / ticksPerRev;

    }

    private double LiniarizedTargetAngle(double targetAngle,double currContinuosAngle){

        double distNormalizedContinuos = Localizer.cwDistance(2*Math.PI - Math.toRadians(AngleOffsetFrom0),targetAngle);

        //RobotInitializers.Dashtelemetry.addData("clockwise distance",distNormalizedContinuos);

        double k = Math.round(
                (currContinuosAngle - distNormalizedContinuos)
                        / (2*Math.PI)
        );

        double candidate =
                distNormalizedContinuos + k * 2*Math.PI;

        if(candidate < distNormalizedContinuos)
            candidate += 2*Math.PI;

        if(candidate > Math.toRadians(MaxAngle))
            return 1e9;
        return currContinuosAngle - candidate;
    }

    public void updateTurretEncoder(SparkFunOTOS.Pose2D robotPose) {

        robotPose = TurretFieldRelativePosition(robotPose);

        double robotHeading = robotPose.h;

        double dx = goalPosition.x - Localizer.getCurrentPosition().x;
        double dy = goalPosition.y - Localizer.getCurrentPosition().y;

        double targetGlobalHeading = Math.atan2(dy, dx);

        //RobotInitializers.Dashtelemetry.addData("targetGlobalHeading",targetGlobalHeading);
        double ShouldHaveTurretHeading = targetGlobalHeading - robotHeading + ManualOffset - Math.toRadians(ShooterCalculator.OffsetTurretAngle);

        //RobotInitializers.Dashtelemetry.addData("manualOffset",ManualOffset);

        //RobotInitializers.Dashtelemetry.addData("ShouldHaveTurretHeading",ShouldHaveTurretHeading);

        double currentTurretRel = fromEncoderToRads();

        //RobotInitializers.Dashtelemetry.addData("currentTurretRel",currentTurretRel);
        //RobotInitializers.Dashtelemetry.addData("currticksTURRET",EncoderBore.getCurrentPosition());

        double error = LiniarizedTargetAngle(ShouldHaveTurretHeading,currentTurretRel);
        globalError = error;

        //RobotInitializers.Dashtelemetry.addData("Error",Math.toDegrees(error));

        RobotLog.ii("Error " , String.valueOf(Math.toDegrees(error)));
        //RobotLog.ii("currpos rads " , String.valueOf(currentTurretRel));

        if(error == 1e9)
            SetTurretPower(0);
        else {

            double TurretFs = 0;
            if(Math.abs(error) > Math.toRadians(3)) {
                turretController.setPidCoefficients(higherrorturretcoefs);
                //TurretFs = FeedForwardTurret;
            }
            else {
                turretController.setPidCoefficients(new PIDCoefficients(higherrorturretcoefs.p * MultiplierPID,higherrorturretcoefs.i,higherrorturretcoefs.d));
                //TurretFs = 0;
            }
            SetTurretPower(turretController.calculatePower(error) + Math.signum(error) * FeedForwardTurret + Kvangular * Localizer.getVelocity().h);
        }
        //SetTurretPower(turretController.calculatePower(error) + Math.signum(error) * FeedForwardTurret + Kvangular * Localizer.getVelocity().h);
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
