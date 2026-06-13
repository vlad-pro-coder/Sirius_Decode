package org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.CloseStart;

import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.Actions.ShootBalls.shootBalls;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.Actions.ShootBalls.shootBallsPreloads;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.Actions.TakeBalls.TakeBallsActionsFromGate;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.Actions.TakeBalls.TakeBallsActionsFromStripsWithJiggle;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.GateCloseWait;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.GoalPositionClose;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.GoingFromCloseStripToShoot;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.GoingFromGateToShoot;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.GoingFromMidStripToShoot;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.GoingFromShootToCloseStrip;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.GoingFromShootToGate;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.GoingFromShootToMidStrip;
import static org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.AutoConstantBLUE.shootingCloseInstant;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
public class BlueClose extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        RobotInitializers.InitializeFull(hardwareMap);

        AsyncScheduler tasks = new AsyncScheduler();
        Shooter shooter = new Shooter();
        Intake intake = new Intake();
        Storage storage = new Storage(intake);
        Turret turret = new Turret();

        Localizer.setPosition(new SparkFunOTOS.Pose2D(530,384,Math.toRadians(42)));


        Turret.goalPosition = GoalPositionClose;
        ShooterCalculator.TurretOffsetMultiplier = -0.55;
        Chassis.setTargetPosition(shootingCloseInstant);

        tasks.AddAnotherAsyncScheduler(shootBallsPreloads(shooter, storage, shootingCloseInstant))
                .AddAnotherAsyncScheduler(TakeBallsActionsFromStripsWithJiggle(storage,GoingFromShootToMidStrip,200,2))
                .AddAnotherAsyncScheduler(shootBalls(storage,shooter,GoingFromMidStripToShoot,200))
                .AddAnotherAsyncScheduler(TakeBallsActionsFromGate(storage,GoingFromShootToGate,GateCloseWait,300,2.0))
                .AddAnotherAsyncScheduler(shootBalls(storage,shooter,GoingFromGateToShoot,600))
                .AddAnotherAsyncScheduler(TakeBallsActionsFromGate(storage,GoingFromShootToGate,GateCloseWait,300,2.2))
                .AddAnotherAsyncScheduler(shootBalls(storage,shooter,GoingFromGateToShoot,600))
                .AddAnotherAsyncScheduler(TakeBallsActionsFromGate(storage,GoingFromShootToGate,GateCloseWait,300,2.8))
                .AddAnotherAsyncScheduler(shootBalls(storage,shooter,GoingFromGateToShoot,600))
                .AddAnotherAsyncScheduler(TakeBallsActionsFromStripsWithJiggle(storage,GoingFromShootToCloseStrip,200,1.5))
                .AddAnotherAsyncScheduler(shootBalls(storage,shooter,GoingFromCloseStripToShoot,100));



        TeleopsStarter.AngleOffsetForDriving = Math.PI/2;

        while(opModeInInit()){
            RobotInitializers.clearCache();
            shooter.UpdateTurretAndPitch();
            telemetry.addData("pos",Localizer.getCurrentPosition());
            storage.update();
            turret.updateTurret(Localizer.getCurrentPosition());
            Localizer.Update();
            telemetry.update();
        }

        waitForStart();

        boolean emergencyPark = true;
        long time = System.currentTimeMillis();

        while(opModeIsActive()){

            RobotInitializers.clearCache();
            RobotInitializers.Dashtelemetry.update();

            RobotInitializers.Dashtelemetry.addData("state Chassis",Chassis.usedTrajectory);
            Localizer.Update();

            tasks.update();
            turret.updateTurret(Localizer.getCurrentPosition());
            shooter.updateFlyWheelSpeed();

            Chassis.update();
            storage.update();
            shooter.UpdateTurretAndPitch();
        }

    }
}
