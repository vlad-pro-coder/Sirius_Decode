package org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.Actions;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.SiriusDecode.ActionsCommandLineImplementation.AsyncScheduler;
import org.firstinspires.ftc.teamcode.SiriusDecode.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.SiriusDecode.Pathing.PurePersuit;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Storage;

import java.util.ArrayList;

public class TakeHumanBalls {

    public static AsyncScheduler TakeHumanBallsActions(Intake intake, Storage storage, SparkFunOTOS.Pose2D takeballspos,SparkFunOTOS.Pose2D takeballposeend)
    {
        return new AsyncScheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        intake.takeIn();
                        Chassis.usedTrajectory = Chassis.trajectoryStates.FREEWILL;
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .DoingTaskUntilSeconds(2,new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setTargetPosition(takeballspos);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(100);
                    }
                })
                .DoingTaskUntilSeconds(3,new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(takeballposeend.x,takeballposeend.y,takeballspos.h));
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(100);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setHeading(takeballposeend.h);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsHeadingDone(5);
                    }
                })
                .DoingTaskUntilSeconds(2,new Task() {
                    @Override
                    protected void Actions() {

                    }

                    @Override
                    protected boolean Conditions() {
                        return storage.CurrentState == Storage.StorageStates.DOWAITACTIONSFORSHOOTING;
                    }
                });
    }
    public static AsyncScheduler TakeHumanBallsActionsAfterGateSlowGo(Intake intake, Storage storage, ArrayList<PurePersuit.Point> GoingToGateToOpen,double r,double PositionDoneTolerance){
        return new AsyncScheduler()
                .StartPurePersuit(GoingToGateToOpen,r)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        intake.takeIn();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(PositionDoneTolerance) || storage.CurrentState == Storage.StorageStates.DOWAITACTIONSFORSHOOTING;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setSlowFollow(true);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })

                .DoingTaskForSecondsWithException(1, new Task() {
                    @Override
                    protected void Actions() {

                    }

                    @Override
                    protected boolean Conditions() {
                        return storage.CurrentState == Storage.StorageStates.DOWAITACTIONSFORSHOOTING;
                    }
                },new Task() {
                    @Override
                    protected void Actions() {

                    }

                    @Override
                    protected boolean Conditions() {
                        return storage.CurrentState != Storage.StorageStates.DOWAITACTIONSFORSHOOTING;
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
    public static AsyncScheduler Buldoze3Times(Intake intake, Storage storage,SparkFunOTOS.Pose2D poseBuldozeReference,boolean Isred){

        return new AsyncScheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions()
                    {

                        Chassis.usedTrajectory = Chassis.trajectoryStates.FREEWILL;
                        intake.takeIn();
                        Chassis.setTargetPosition(poseBuldozeReference);

                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(600) || storage.CurrentState == Storage.StorageStates.DOWAITACTIONSFORSHOOTING;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.Heading.setPidCoefficients(Chassis.HeadingVibrate);
                        Chassis.setSlowFollow(true);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .DoingTaskForSecondsWithException(1, new Task() {
                    @Override
                    protected void Actions() {

                    }

                    @Override
                    protected boolean Conditions() {
                        return storage.CurrentState == Storage.StorageStates.DOWAITACTIONSFORSHOOTING;
                    }
                }, new Task() {
                    @Override
                    protected void Actions() {

                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        intake.spit();
                        Chassis.Heading.setPidCoefficients(Chassis.HeadingNormal);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })

                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setSlowFollow(false);
                        if(Isred)
                                Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(poseBuldozeReference.x-400,poseBuldozeReference.y+600,poseBuldozeReference.h ));
                        else
                            Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(poseBuldozeReference.x+400,poseBuldozeReference.y+600,poseBuldozeReference.h ));
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(100) || storage.CurrentState == Storage.StorageStates.DOWAITACTIONSFORSHOOTING;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        intake.takeIn();
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(poseBuldozeReference.x,poseBuldozeReference.y+600,poseBuldozeReference.h ));
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(600) || storage.CurrentState == Storage.StorageStates.DOWAITACTIONSFORSHOOTING;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setSlowFollow(true);
                        Chassis.Heading.setPidCoefficients(Chassis.HeadingVibrate);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .DoingTaskForSecondsWithException(1, new Task() {
                    @Override
                    protected void Actions() {

                    }

                    @Override
                    protected boolean Conditions() {
                        return storage.CurrentState == Storage.StorageStates.DOWAITACTIONSFORSHOOTING;
                    }
                }, new Task() {
                    @Override
                    protected void Actions() {

                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setSlowFollow(false);
                        Chassis.Heading.setPidCoefficients(Chassis.HeadingNormal);
                        intake.spit();
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                });



    }

    public static AsyncScheduler ComeAndWaitForBalls(Intake intake, Storage storage,SparkFunOTOS.Pose2D takeballspostobefeeded){
        return new AsyncScheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setHeading(takeballspostobefeeded.h);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsHeadingDone(12);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        intake.takeIn();
                        Chassis.setTargetPosition(takeballspostobefeeded);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(80);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {

                    }

                    @Override
                    protected boolean Conditions() {
                        return storage.CurrentState == Storage.StorageStates.DOWAITACTIONSFORSHOOTING;
                    }
                });

    }
}
