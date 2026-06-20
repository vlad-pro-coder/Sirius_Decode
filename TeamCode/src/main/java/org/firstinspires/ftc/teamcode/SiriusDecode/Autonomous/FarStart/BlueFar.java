package org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.FarStart;

import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.Actions.ShootBalls.shootBalls;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.Actions.ShootBalls.shootBallsPreloads;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.Actions.TakeBalls.TakeBallsActionsFromStrips;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.Actions.TakeBalls.TakeBallsActionsFromStripsWithJiggle;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.GoalPositionFar;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.GoingFromFarStripToShoot;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.GoingFromGatherToShoot;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.GoingFromHumanToShoot;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.GoingFromShootToFarStrip;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.GoingFromShootToGatherBalls;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.GoingFromShootToHuman;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.shootingFarInstant;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SiriusDecode.ActionsCommandLineImplementation.AsyncScheduler;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Shooter;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.ShooterCalculator;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Storage;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Turret;
import org.firstinspires.ftc.teamcode.SiriusDecode.TeleopsStarter;


@Autonomous
@Config
public class BlueFar extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RobotInitializers.InitializeFull(hardwareMap);

        AsyncScheduler tasks = new AsyncScheduler();
        Shooter shooter = new Shooter();
        Intake intake = new Intake();
        Storage storage = new Storage(intake);
        Turret turret = new Turret();
        //AprilTagVision apriltagHandler = new AprilTagVision(hardwareMap,"Webcam 1");

        //apriltagHandler.open();

        Turret.goalPosition = GoalPositionFar;
        Chassis.usedTrajectory = Chassis.trajectoryStates.FREEWILL;


        Chassis.setTargetPosition(shootingFarInstant);

        ShooterCalculator.TurretOffsetMultiplier = -0.3;


        tasks.AddAnotherAsyncScheduler(shootBallsPreloads(shooter,storage, shootingFarInstant))
                .AddAnotherAsyncScheduler(TakeBallsActionsFromStripsWithJiggle(storage, GoingFromShootToFarStrip,300,1))
                .AddAnotherAsyncScheduler(shootBalls(storage,shooter,GoingFromFarStripToShoot,300))
                .AddAnotherAsyncScheduler(TakeBallsActionsFromStripsWithJiggle(storage, GoingFromShootToHuman, 200,2))
                .AddAnotherAsyncScheduler(shootBalls(storage,shooter,GoingFromHumanToShoot,300))

                .AddAnotherAsyncScheduler(TakeBallsActionsFromStrips(storage, GoingFromShootToGatherBalls, 500,1.5))
                .AddAnotherAsyncScheduler(shootBalls(storage,shooter,GoingFromGatherToShoot,300))

                .AddAnotherAsyncScheduler(TakeBallsActionsFromStrips(storage, GoingFromShootToGatherBalls, 500,1.5))
                .AddAnotherAsyncScheduler(shootBalls(storage,shooter,GoingFromGatherToShoot,300))

                .AddAnotherAsyncScheduler(TakeBallsActionsFromStrips(storage, GoingFromShootToGatherBalls, 500,1.5))
                .AddAnotherAsyncScheduler(shootBalls(storage,shooter,GoingFromGatherToShoot,300))

                .AddAnotherAsyncScheduler(TakeBallsActionsFromStrips(storage, GoingFromShootToGatherBalls, 500,1.5))
                .AddAnotherAsyncScheduler(shootBalls(storage,shooter,GoingFromGatherToShoot,300))

                .AddAnotherAsyncScheduler(TakeBallsActionsFromStrips(storage, GoingFromShootToGatherBalls, 500,1.5))
                .AddAnotherAsyncScheduler(shootBalls(storage,shooter,GoingFromGatherToShoot,300));

        TeleopsStarter.AngleOffsetForDriving = -Math.PI/2;

        while(opModeInInit()){
            RobotInitializers.clearCache();
            shooter.UpdateTurretAndPitch();
            telemetry.addData("pos", Localizer.getCurrentPosition());
            Localizer.Update();
            storage.update();

            Turret.EncoderBore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Turret.EncoderBore.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turret.updateTurret(Localizer.getCurrentPosition());
            telemetry.addData("localizer",Localizer.getCurrentPosition());
            telemetry.update();
        }

        waitForStart();
        boolean emergencyPark = true;
        long time = System.currentTimeMillis();

        while(opModeIsActive()){
            RobotInitializers.clearCache();
            RobotInitializers.Dashtelemetry.update();
            tasks.update();
            Localizer.Update();

            /*if(!storage.HasMotif() && TryAndReadMotif){
                cameraTask.AddAnotherAsyncScheduler(ReadMotif(apriltagHandler,GoalPositionClose,ObeliscFar));
                TryAndReadMotif = false;
            }*/



            shooter.UpdateTurretAndPitch();
            turret.updateTurret(Localizer.getCurrentPosition());
            shooter.updateFlyWheelSpeed();
            Chassis.update();
            storage.update();
            /*if((double) (System.currentTimeMillis() - time) / 1000 >= 29.4 && emergencyPark){
                emergencyPark = false;
                tasks.clearAsyncQueues();
                tasks.clear();
                Chassis.usedTrajectory = Chassis.trajectoryStates.FREEWILL;
                storage.tasks.clear();
                Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(shootingFarFromStrips.x+400, shootingFarFromStrips.y+200,shootingClose.h));
            }*/
        }

    }
}
