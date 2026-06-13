package org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.Actions;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.SiriusDecode.ActionsCommandLineImplementation.AsyncScheduler;
import org.firstinspires.ftc.teamcode.SiriusDecode.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.SiriusDecode.Pathing.PurePersuit;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Storage;

import java.util.ArrayList;

public class TakeBalls {
    public static AsyncScheduler TakeBallsActionsFromStrips(Storage storage, ArrayList<PurePersuit.Point> path, double radius, double timeout){
        return new AsyncScheduler()
                .StartPurePersuit(path,radius)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {

                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(500);
                    }
                })
                .DoingTaskUntilSeconds(timeout,new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setSlowFollow(true);
                    }

                    @Override
                    protected boolean Conditions() {
                        return storage.CurrentState == Storage.StorageStates.DOWAITACTIONSFORSHOOTING;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setSlowFollow(false);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                });
    }
    public static AsyncScheduler TakeBallsActionsFromStripsWithJiggle(Storage storage, ArrayList<PurePersuit.Point> path, double radius, double timeout){
        return new AsyncScheduler()
                .StartPurePersuit(path,radius)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {

                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(500);
                    }
                })
                .DoingTaskUntilSeconds(timeout,new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setSlowFollow(true);
                    }

                    @Override
                    protected boolean Conditions() {
                        return storage.CurrentState == Storage.StorageStates.DOWAITACTIONSFORSHOOTING || Chassis.IsPositionDone(100);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.Heading.setPidCoefficients(Chassis.HeadingVibrate);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .DoingTaskUntilSeconds(timeout,new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setSlowFollow(true);
                    }

                    @Override
                    protected boolean Conditions() {
                        return storage.CurrentState == Storage.StorageStates.DOWAITACTIONSFORSHOOTING;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setSlowFollow(false);
                        Chassis.Heading.setPidCoefficients(Chassis.HeadingNormal);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                });
    }

    public static AsyncScheduler TakeBallsActionsFromGate(Storage storage,ArrayList<PurePersuit.Point> GoingToGateToOpen, SparkFunOTOS.Pose2D poseToCollect,double radius ,double timeoutGate){
        return new AsyncScheduler()

                .StartPurePersuit(GoingToGateToOpen,radius)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {

                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(200);
                    }
                })
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.usedTrajectory = Chassis.trajectoryStates.FREEWILL;
                        Chassis.setTargetPosition(poseToCollect);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .DoingTaskUntilSeconds(timeoutGate, new Task() {
                    @Override
                    protected void Actions() {

                    }

                    @Override
                    protected boolean Conditions() {
                        return storage.CurrentState == Storage.StorageStates.DOWAITACTIONSFORSHOOTING;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        storage.CurrentState = Storage.StorageStates.DOWAITACTIONSFORSHOOTING;
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                });


    }

}
