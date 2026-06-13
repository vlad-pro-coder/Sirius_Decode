package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Shooter;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.ShooterCalculator;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.ShooterCalculatorWithDots;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Storage;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Turret;

@TeleOp
@Config
public class TuneShootingDotValues extends LinearOpMode {


    public static int id = 0;

    public static int row = 10, column = 25;

    public static int idangle = 0;

    public static double RPM = 0;
    public static double hood = 50;
    public static double hoodCorection = 0;

    public static boolean Shoot = false;

    public static double lastrow,lastcolumn,lastangle;


    @Override
    public void runOpMode() throws InterruptedException {


        RobotInitializers.InitializeFull(hardwareMap);
        ShooterCalculatorWithDots shootcalc = new ShooterCalculatorWithDots(Turret.goalPosition.x,Turret.goalPosition.y,3200,3200,100,-1,-1,4);//goal red close
        Shooter shooter = new Shooter();
        ShooterCalculator shooterCalculator = new ShooterCalculator();
        Intake intake = new Intake();
        Storage storage = new Storage(intake);

        Turret turret = new Turret();
        waitForStart();

        while(opModeIsActive())
        {
            RobotInitializers.clearCache();
            Localizer.Update();
            int spatialId = shootcalc.getNearestSpatialId(Localizer.getCurrentPosition().x, Localizer.getCurrentPosition().y);
            int angleIdx = shootcalc.getNearestAngleIndex(Math.toDegrees(Localizer.normalizeTo0To2PI(Localizer.getCurrentPosition().h)));
            RobotInitializers.Dashtelemetry.addData("curr dot ID",shootcalc.getCombinedId(spatialId,angleIdx));
            RobotInitializers.Dashtelemetry.addData("data about point ",shootcalc.getPoseFromCombinedId(shootcalc.getCombinedId(spatialId,angleIdx)));
            shootcalc.printTunedDots();

            RobotInitializers.Dashtelemetry.addData("data about point selected",shootcalc.getPoseFromCombinedId(shootcalc.getCombinedId(shootcalc.FromRowAndColumnGetSpatialId(row,column),idangle)));

            if(lastrow != row || lastcolumn != column || lastangle != idangle) {
                Chassis.setTargetPosition(shootcalc.getPoseFromCombinedId(shootcalc.getCombinedId(shootcalc.FromRowAndColumnGetSpatialId(row, column), idangle)));
                lastrow = row;
                lastcolumn = column;
                lastangle = idangle;
            }
            Chassis.update();
            double newHood = shooterCalculator.CompensateRPMSagWitchHood(shooter.RPMError(),hood,hoodCorection);
            shooter.SetAngleToHood(newHood);
            shooter.SetTargetRPM(RPM);

            shooter.updateFlyWheelSpeed();
            storage.update();

            if(Shoot){
                storage.CurrentState = Storage.StorageStates.DOSHOOTACTIONS;
                Shoot = false;
            }

            turret.updateTurret(Localizer.getCurrentPosition());
        }
    }
}
