package org.firstinspires.ftc.teamcode.SiriusDecode.ActionsCommandLineImplementation;

public abstract class Task {

    public boolean RanOnce = false;
    public final boolean Run() {
        if(!RanOnce){
            RanOnce = true;
            Actions();
        }

        if(Conditions()) {
            RanOnce = false;
            return true;
        }
        return false;
    }

    // You override this instead of Run()
    protected abstract void Actions();

    // Called repeatedly after setup
    protected abstract boolean Conditions();
}
