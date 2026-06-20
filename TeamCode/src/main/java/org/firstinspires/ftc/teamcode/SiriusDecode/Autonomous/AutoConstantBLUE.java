package org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.SiriusDecode.Pathing.PurePersuit;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class AutoConstantBLUE {

    /*auto far constants */

    public static double ToleranceHuman = 1500;
    public static SparkFunOTOS.Pose2D
            FarStripBallsGoal = new SparkFunOTOS.Pose2D(-1381,689,Math.toRadians(90)),
            HumanPlayerBalls = new SparkFunOTOS.Pose2D(-1340,50,Math.toRadians(90))
    ;

    public static SparkFunOTOS.Pose2D shootingFarInstant = new SparkFunOTOS.Pose2D(-230,101,Math.toRadians(42));
    public static SparkFunOTOS.Pose2D GoalPositionFar = new SparkFunOTOS.Pose2D(-1320,3290,0);

    public static ArrayList<PurePersuit.Point> GoingFromShootToFarStrip = new ArrayList<>(Arrays.asList(
            new PurePersuit.Point(shootingFarInstant.x,shootingFarInstant.y, shootingFarInstant.h ,0.2),
            new PurePersuit.Point(-448, 669, Math.toRadians(58), 0.2),
            new PurePersuit.Point(FarStripBallsGoal.x, FarStripBallsGoal.y, FarStripBallsGoal.h, 0.2)
    ));


    public static ArrayList<PurePersuit.Point> GoingFromFarStripToShoot = new ArrayList<>(Arrays.asList(
            new PurePersuit.Point(FarStripBallsGoal.x, FarStripBallsGoal.y, FarStripBallsGoal.h, 0.2),
            new PurePersuit.Point(shootingFarInstant.x, shootingFarInstant.y,Math.toDegrees(68) , 0.2)
    ));


    public static ArrayList<PurePersuit.Point> GoingFromShootToHuman = new ArrayList<>(Arrays.asList(
            new PurePersuit.Point(shootingFarInstant.x,shootingFarInstant.y, Math.toDegrees(68) ,0.2),
            new PurePersuit.Point(-633, 64, Math.toRadians(105), 0.2),
            new PurePersuit.Point(HumanPlayerBalls.x, HumanPlayerBalls.y, HumanPlayerBalls.h, 0.2)
    ));


    public static ArrayList<PurePersuit.Point> GoingFromHumanToShoot = new ArrayList<>(Arrays.asList(
            new PurePersuit.Point(HumanPlayerBalls.x, HumanPlayerBalls.y, HumanPlayerBalls.h, 0.2),
            new PurePersuit.Point(-423, 50,HumanPlayerBalls.h , 0.2)
    ));

    public static ArrayList<PurePersuit.Point> GoingFromShootToGatherBalls = new ArrayList<>(Arrays.asList(
            new PurePersuit.Point(-423, -129,HumanPlayerBalls.h, 0.2),
            new PurePersuit.Point(HumanPlayerBalls.x-300, HumanPlayerBalls.y, HumanPlayerBalls.h-Math.toRadians(20), 0.5),
            new PurePersuit.Point(-1300, 1200,Math.toRadians(37), 0.4)
            ));

    public static ArrayList<PurePersuit.Point> GoingFromGatherToShoot = new ArrayList<>(Arrays.asList(
            new PurePersuit.Point(-1289, 640,Math.toRadians(45), 0.2),
            new PurePersuit.Point(-193, 86,Math.toRadians(63), 0.2)
            ));



    /*auto close constants*/
    public static SparkFunOTOS.Pose2D CloseStripBallsToGoalCloseStart = new SparkFunOTOS.Pose2D(600,1170,Math.toRadians(-90)),
            MiddleStripBallsGoalCloseStart = new SparkFunOTOS.Pose2D(860,1890,Math.toRadians(-90)),
            GateClose = new SparkFunOTOS.Pose2D(630,1780,Math.toRadians(-90)),

    GateCloseWait = new SparkFunOTOS.Pose2D(850,1895,Math.toRadians(-127));

    public static SparkFunOTOS.Pose2D shootingClose = new SparkFunOTOS.Pose2D(-452,1440,Math.toRadians(-73));
    public static SparkFunOTOS.Pose2D shootingCloseInstant = new SparkFunOTOS.Pose2D(-452,1440,Math.toRadians(41));

    public static ArrayList<PurePersuit.Point> GoingFromShootToMidStrip = new ArrayList<>(Arrays.asList(
            new PurePersuit.Point(shootingCloseInstant.x,shootingCloseInstant.y,Math.toRadians(-30) ,0.2),
            new PurePersuit.Point(-21, 1777, Math.toRadians(-60), 0.2),
            new PurePersuit.Point(MiddleStripBallsGoalCloseStart.x, MiddleStripBallsGoalCloseStart.y, MiddleStripBallsGoalCloseStart.h, 0.2)
    ));


    public static ArrayList<PurePersuit.Point> GoingFromMidStripToShoot = new ArrayList<>(Arrays.asList(
            new PurePersuit.Point(MiddleStripBallsGoalCloseStart.x, MiddleStripBallsGoalCloseStart.y, MiddleStripBallsGoalCloseStart.h, 0.4),
            new PurePersuit.Point(-21, 1777, MiddleStripBallsGoalCloseStart.h- Math.toRadians(-40), 0.4),
            new PurePersuit.Point(shootingClose.x,shootingClose.y,shootingClose.h,0.4)
    ));

    public static ArrayList<PurePersuit.Point> GoingFromShootToCloseStrip = new ArrayList<>(Arrays.asList(
            new PurePersuit.Point(shootingClose.x,shootingClose.y, CloseStripBallsToGoalCloseStart.h ,0.2),
            new PurePersuit.Point(CloseStripBallsToGoalCloseStart.x, CloseStripBallsToGoalCloseStart.y, CloseStripBallsToGoalCloseStart.h-Math.toRadians(10), 0.4)
    ));


    public static ArrayList<PurePersuit.Point> GoingFromCloseStripToShoot = new ArrayList<>(Arrays.asList(
            new PurePersuit.Point(CloseStripBallsToGoalCloseStart.x, CloseStripBallsToGoalCloseStart.y, CloseStripBallsToGoalCloseStart.h, 0.4),
            new PurePersuit.Point(-470,785, Math.toRadians(-90) ,0.4)
    ));

    public static ArrayList<PurePersuit.Point> GoingFromShootToGate = new ArrayList<>(Arrays.asList(
            new PurePersuit.Point(shootingClose.x, shootingClose.y, GateClose.h, 0.2),
            new PurePersuit.Point(100, 1708, GateClose.h,0.2),
            new PurePersuit.Point(GateClose.x, GateClose.y, GateClose.h ,0.2)
    ));

    public static ArrayList<PurePersuit.Point> GoingFromGateToShoot = new ArrayList<>(Arrays.asList(
            new PurePersuit.Point(GateCloseWait.x, GateCloseWait.y, GateClose.h ,0.3),
            new PurePersuit.Point(-100, 1708, MiddleStripBallsGoalCloseStart.h, 0.2),
            new PurePersuit.Point(shootingClose.x, shootingClose.y, shootingClose.h, 0.2)
    ));

    public static SparkFunOTOS.Pose2D GoalPositionClose = new SparkFunOTOS.Pose2D(780,50,0);


}
