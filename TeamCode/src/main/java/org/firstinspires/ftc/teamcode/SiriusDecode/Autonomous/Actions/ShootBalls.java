package org.firstinspires.ftc.teamcode.SiriusDecode.Autonomous.Actions;

import static org.firstinspires.ftc.teamcode.SiriusDecode.TeleopsStarter.shooter;
import static org.firstinspires.ftc.teamcode.SiriusDecode.TeleopsStarter.storage;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.SiriusDecode.ActionsCommandLineImplementation.AsyncScheduler;
import org.firstinspires.ftc.teamcode.SiriusDecode.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.SiriusDecode.Pathing.PurePersuit;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Shooter;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Storage;

import java.util.ArrayList;

public class ShootBalls {
    public static AsyncScheduler shootBalls(Storage storage,Shooter shooter, ArrayList<PurePersuit.Point> PurePersuitPath,double radius){
        return new AsyncScheduler()
                .StartPurePersuit(PurePersuitPath,radius)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        storage.CurrentState = Storage.StorageStates.DOWAITACTIONSFORSHOOTING;
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(600);
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
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        RobotInitializers.Dashtelemetry.addLine("not going to shoot");
                    }

                    @Override
                    protected boolean Conditions() {
                        return shooter.RPMError(200) && Chassis.IsPositionDone(80) && Localizer.getVelocity().x < 100 && Localizer.getVelocity().y < 100;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        storage.CurrentState = Storage.StorageStates.DOSHOOTACTIONS;
                        RobotLog.d("DUMP");
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.4)
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
    public static AsyncScheduler shootBallsPreloads(Shooter shooter,Storage storage, SparkFunOTOS.Pose2D shootPos){
        return new AsyncScheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.usedTrajectory = Chassis.trajectoryStates.FREEWILL;
                        Chassis.setTargetPosition(shootPos);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })

                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(600);
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
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                    }

                    @Override
                    protected boolean Conditions() {
                        return shooter.RPMError(200) && Chassis.IsPositionDone(80) && Localizer.getVelocity().x < 100 && Localizer.getVelocity().y < 100;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        storage.CurrentState = Storage.StorageStates.DOSHOOTACTIONS;
                        RobotLog.d("DUMP");
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.4)
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

    public static AsyncScheduler shootBalls(Storage storage, Shooter shooter, Intake intake, SparkFunOTOS.Pose2D poseToShoot){
        return new AsyncScheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        intake.spit();
                        Chassis.setSlowFollow(false);
                        Chassis.usedTrajectory = Chassis.trajectoryStates.FREEWILL;
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setTargetPosition(poseToShoot);
                        RobotLog.d("heading wait");
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(400);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setSlowFollow(true);
                        intake.takeIn(0.4);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(200);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        RobotInitializers.Dashtelemetry.addLine("waiting for speed");
                        RobotLog.d("flywheel wait");
                    }

                    @Override
                    protected boolean Conditions() {
                        return shooter.RPMError(250);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        storage.CurrentState = Storage.StorageStates.DOSHOOTACTIONS;
                        RobotLog.d("DUMP");
                    }

                    @Override
                    protected boolean Conditions() {
                        return storage.CurrentState == Storage.StorageStates.GOBALL1;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        intake.stop();
                        Chassis.setSlowFollow(false);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                });
    }
}
