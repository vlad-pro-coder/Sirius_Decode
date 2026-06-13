package org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents;


import static org.firstinspires.ftc.teamcode.SiriusDecode.TeleopsStarter.gm1;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SiriusDecode.ActionsCommandLineImplementation.AsyncScheduler;
import org.firstinspires.ftc.teamcode.SiriusDecode.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.SiriusDecode.MathHelpers.Colors;
import org.firstinspires.ftc.teamcode.SiriusDecode.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.CRServoPlus;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.RGBsensor;
import org.firstinspires.ftc.teamcode.SiriusDecode.Wrapers.ServoPlus;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class Storage {

    public static CRServoPlus StorageMotor;
    public static RGBsensor colorsensor;
    public static AnalogInput AbsoluteEncoder;
    public static ServoPlus Putulica;
    public static DigitalChannel sensor;

    public static double collect1 = 45,collect2=165,collect3=285,WaitingForShootStoragePos = 55;
    public static double sculatPos = 305,blegPos = 210;
    public static boolean ProgramActivatedIntake = false;
    /*public static ArrayList<Colors.ColorType> Motif = new ArrayList<>(
            Arrays.asList(
                    Colors.ColorType.PURPLE,
                    Colors.ColorType.PURPLE,
                    Colors.ColorType.PURPLE
            )
    ),
            CurrentColors = new ArrayList<>(
                    Arrays.asList(
                            Colors.ColorType.NONE,
                            Colors.ColorType.NONE,
                            Colors.ColorType.NONE
                    )
            );*/
    public boolean GivenWaitActions = false;


    public PIDController storagecontroller = new PIDController(0.009,0,0.0007);
    public PIDCoefficients origin_coefs = new PIDCoefficients(0.009,0,0.0007);

    public static double TimeTransfer = 0.5;
    public ElapsedTime timeoutForIntake = new ElapsedTime()
        ,ContinuosSpindexRotateTimer = new ElapsedTime()
        ,ImpactCode = new ElapsedTime()
        ,SpeedTimer = new ElapsedTime();
            ;
    public AsyncScheduler tasks;
    public static boolean sort = false;

    public StorageStates LastBallAction;
    public StorageStates CurrentState;
    public double target = 0;


    public enum StorageStates{
            WAITANDDETECT,
            GOBALL1,
            GOBALL2,
            GOBALL3,
            DOWAITACTIONSFORSHOOTING,
            DOSHOOTACTIONS,

    }

    public AsyncScheduler GoToASpindexPos(double pos){
        return new AsyncScheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        intake.takeIn(0.7);
                        ProgramActivatedIntake = true;
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.03)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        target = pos;
                    }

                    @Override
                    protected boolean Conditions() {
                        return SpindexError() < 100;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {

                        ProgramActivatedIntake = false;
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                });
    }

    public AsyncScheduler GoPosThenStopAndPower(){
        return new AsyncScheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        ProgramActivatedIntake = true;
                        target = target;
                        target += target < 0? 360:0;
                    }

                    @Override
                    protected boolean Conditions() {
                        StorageMotor.setPower(storagecontroller.calculatePower(SpindexError()));
                        return SpindexError() < 5;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        setPowerEquilibrated(0);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        setPowerEquilibrated(1);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.05)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        target = target - 120;
                        target += target < 0? 360:0;
                    }

                    @Override
                    protected boolean Conditions() {
                        StorageMotor.setPower(storagecontroller.calculatePower(SpindexError()));
                        return SpindexError() < 5;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        setPowerEquilibrated(0);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        setPowerEquilibrated(1);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.05)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        target = target - 120;
                        target += target < 0? 360:0;
                    }

                    @Override
                    protected boolean Conditions() {
                        StorageMotor.setPower(storagecontroller.calculatePower(SpindexError()));
                        return SpindexError() < 5;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        setPowerEquilibrated(0);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        setPowerEquilibrated(1);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.1);
    }
    public AsyncScheduler GoPosThenStopAndPowerSorted(){
        return new AsyncScheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        target = target - 10;
                        target += target < 0? 360:0;
                    }

                    @Override
                    protected boolean Conditions() {
                        StorageMotor.setPower(storagecontroller.calculatePower(SpindexError()));
                        return SpindexError() < 2;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        setPowerEquilibrated(1);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.02)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        target = target - 120;
                        target += target < 0? 360:0;
                    }

                    @Override
                    protected boolean Conditions() {
                        StorageMotor.setPower(storagecontroller.calculatePower(SpindexError()));
                        return SpindexError() < 2;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        setPowerEquilibrated(0);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.5)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        setPowerEquilibrated(1);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.02)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        target = target - 120;
                        target += target < 0? 360:0;
                    }

                    @Override
                    protected boolean Conditions() {
                        StorageMotor.setPower(storagecontroller.calculatePower(SpindexError()));
                        return SpindexError() < 2;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        setPowerEquilibrated(0);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.5)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        setPowerEquilibrated(1);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.02);
    }
    public AsyncScheduler GoToASpindexPosAndErect(double val){
        return new AsyncScheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        intake.takeIn(0.7);
                        ProgramActivatedIntake = true;
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.07)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        target = val;
                        storagecontroller.setPidCoefficients(new PIDCoefficients(origin_coefs.p * 1.5, origin_coefs.i, origin_coefs.d * 1.5));
                    }

                    @Override
                    protected boolean Conditions() {
                        return SpindexError() < 6;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        storagecontroller.setPidCoefficients(origin_coefs);
                        Putulica.setAngle(sculatPos);
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
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.15)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        intake.stop();
                        ProgramActivatedIntake = false;
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                });
    }

    Intake intake;

    public Storage(Intake intake){
        this.intake = intake;
        CurrentState = StorageStates.GOBALL1;
        storagecontroller.setTargetPosition(0);
        tasks = new AsyncScheduler();
    }

    public double FromVtoPi(){
        return AbsoluteEncoder.getVoltage() / AbsoluteEncoder.getMaxVoltage() * 360;
    }

    double currspeed=0;
    double lastspeed=0;
    public void SpindexSpeed(){
        this.currspeed = (FromVtoPi() - this.lastspeed) /  SpeedTimer.seconds();
        this.lastspeed = FromVtoPi();
        SpeedTimer.reset();
    }

    private void setPowerEquilibrated(double power){
        double p = 12.8 / RobotInitializers.VOLTAGE;
        StorageMotor.setPower(power * p);
    }

    public double SpindexError(){
        double error = target - FromVtoPi();

        // Wrap into [-PI, PI]
        error = (error + 180) % (2 * 180);

        if (error < 0)
            error += 2 * 180;

        error -= 180;

        return error;
    }

    /*public double ValidPosToSort(){
        if(CurrentColors.get(0) == Motif.get(0) && CurrentColors.get(1) == Motif.get(1) && CurrentColors.get(2) == Motif.get(2))
            return collect1;
        else if(CurrentColors.get(1) == Motif.get(0) && CurrentColors.get(2) == Motif.get(1) && CurrentColors.get(0) == Motif.get(2))
            return collect2;
        else if(CurrentColors.get(2) == Motif.get(0) && CurrentColors.get(0) == Motif.get(1) && CurrentColors.get(1) == Motif.get(2))
            return collect3;
        else
            return -1;
    }*/

    public void update(){

        /*RobotInitializers.Dashtelemetry.addData("currstate",CurrentState);
        RobotInitializers.Dashtelemetry.addData("spindexerror",SpindexError());
        RobotInitializers.Dashtelemetry.addData("distancesensor",colorsensor.getDistance(DistanceUnit.CM));

        RobotInitializers.Dashtelemetry.addData("Lastballaction",LastBallAction);

        RobotInitializers.Dashtelemetry.addData("going next",colorsensor.getDistance(DistanceUnit.CM) < 4.5 && Math.abs(SpindexError()) < 20 && tasks.IsSchedulerDone());
        RobotInitializers.Dashtelemetry.addData("speed spindex",currspeed);
        */
        SpindexSpeed();

        switch (CurrentState){
            case WAITANDDETECT:
                if(!sensor.getState() && tasks.IsSchedulerDone()){
                    GivenWaitActions = false;
                    switch (LastBallAction){
                        case GOBALL1:
                            CurrentState = StorageStates.GOBALL2;
                            break;
                        case GOBALL2:
                            CurrentState = StorageStates.GOBALL3;
                            break;
                        case GOBALL3:
                            CurrentState = StorageStates.DOWAITACTIONSFORSHOOTING;
                            break;
                    }
                }
                break;
            case GOBALL1:
                tasks.
                        addTask(new Task() {
                            @Override
                            protected void Actions() {
                                Putulica.setAngle(blegPos);
                            }

                            @Override
                            protected boolean Conditions() {
                                return true;
                            }
                        });
                tasks.AddAnotherAsyncScheduler(GoToASpindexPos(collect1));
                LastBallAction = StorageStates.GOBALL1;
                CurrentState = StorageStates.WAITANDDETECT;
                break;
            case GOBALL2:
                tasks.AddAnotherAsyncScheduler(GoToASpindexPos(collect2));
                LastBallAction = StorageStates.GOBALL2;
                CurrentState = StorageStates.WAITANDDETECT;
                break;
            case GOBALL3:
                tasks.AddAnotherAsyncScheduler(GoToASpindexPos(collect3));
                LastBallAction = StorageStates.GOBALL3;
                CurrentState = StorageStates.WAITANDDETECT;
                break;
            case DOWAITACTIONSFORSHOOTING:
                if(CurrentState != LastBallAction) {
                    tasks.AddAnotherAsyncScheduler(GoToASpindexPosAndErect(target + WaitingForShootStoragePos));
                    LastBallAction = StorageStates.DOWAITACTIONSFORSHOOTING;
                    if(gm1 != null)
                        gm1.rumble(100);
                }
                break;
            case DOSHOOTACTIONS:
                if(CurrentState != LastBallAction) {
                    LastBallAction = StorageStates.DOSHOOTACTIONS;
                    tasks.AddAnotherAsyncScheduler(GoPosThenStopAndPower());
                    //ContinuosSpindexRotateTimer.reset();
                    ProgramActivatedIntake = true;
                }
                intake.takeIn();
                /*if(ContinuosSpindexRotateTimer.seconds() < TimeTransfer) {
                    intake.takeIn();
                    StorageMotor.setPower(1);
                    ProgramActivatedIntake = true;
                }*/

                if(tasks.IsSchedulerDone()/*ContinuosSpindexRotateTimer.seconds() >= TimeTransfer*/ && LastBallAction == StorageStates.DOSHOOTACTIONS) {
                    StorageMotor.setPower(0);
                    intake.stop();
                    ProgramActivatedIntake = false;
                    CurrentState = StorageStates.GOBALL1;
                }

        }

        /*if(timeoutForIntake.seconds() < 0.3) {
            intake.takeIn(0.8);
            ProgramActivatedIntake = true;
        }
        else if(CurrentState != StorageStates.DOSHOOTACTIONS)
            ProgramActivatedIntake = false;
         */

        if(CurrentState != StorageStates.DOSHOOTACTIONS)
                StorageMotor.setPower(storagecontroller.calculatePower(SpindexError()));
        tasks.update();


    }
}
