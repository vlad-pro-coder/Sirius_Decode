    package org.firstinspires.ftc.teamcode.SiriusDecode.Pathing;

    import com.acmerobotics.dashboard.config.Config;
    import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

    import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Localizer;
    import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;

    import java.util.ArrayList;

    @Config
    public class MotionPredictivControl {

        public static double delta_t = 0.1;
        // note: the C++ you provided updated U by subtracting costDeriv (no explicit learning rate).
        // if you want a learning rate, re-introduce it and multiply the derivatives by it.
        public static int MaxCorrectionInter = 10;
        public static double PositionalModifier = 0.3;
        public static double AngularModifier = 0.5;
        public static double VelocityPositionalModifier = 0.3;
        public static double VelocityAngularModifier = 0.5;
        public static double Friction = 0.0;

        public SparkFunOTOS.Pose2D target = new SparkFunOTOS.Pose2D(0,0,0); // mm and rads
        private ArrayList<SparkFunOTOS.Pose2D> bestU = new ArrayList<>();
        private double mini = 1e9;
        public int FutureExactPredictions = 10;

        public static double MaxVelocityH = Math.toRadians(90.0);
        public static double MaxVelocityX = 1500.0; // mm/sec
        public static double MaxVelocityY = 1500.0; // mm/sec
        public static double MaxAccelerationH = Math.toRadians(15.0);
        public static double MaxAccelerationX = 100.0;
        public static double MaxAccelerationY = 100.0;

        // warm start tracking
        private boolean FirstIteration = false;

        public MotionPredictivControl(int FuturePredictions) {
            this.FutureExactPredictions = FuturePredictions;
            this.target = new SparkFunOTOS.Pose2D(0, 0, 0);
        }

        // initial guess (repeated current velocity)
        private ArrayList<SparkFunOTOS.Pose2D> GeneralGuessU() {
            ArrayList<SparkFunOTOS.Pose2D> U = new ArrayList<>();
            SparkFunOTOS.Pose2D vNow = Localizer.getVelocity(); // world-frame velocity
            for (int i = 0; i < FutureExactPredictions; i++) {
                U.add(new SparkFunOTOS.Pose2D(vNow.x, vNow.y, vNow.h));
            }
            return U;
        }

        // warm-start guess using previous bestU (shifted)
        private ArrayList<SparkFunOTOS.Pose2D> GeneralGuessUWithWarm() {
            ArrayList<SparkFunOTOS.Pose2D> U = new ArrayList<>();
            if (bestU == null || !FirstIteration) {
                FirstIteration = true;
                return GeneralGuessU();
            }
            // push bestU[1..end] and repeat last for final slot to keep same size
            for (int i = 1; i < bestU.size(); i++) {
                SparkFunOTOS.Pose2D u = bestU.get(i);
                U.add(new SparkFunOTOS.Pose2D(u.x, u.y, u.h));
            }
            // repeat last element to keep size consistent
            SparkFunOTOS.Pose2D last = bestU.get(bestU.size() - 1);
            U.add(new SparkFunOTOS.Pose2D(last.x, last.y, last.h));
            // if bestU was smaller than FutureExactPredictions (shouldn't happen) fill remaining with current vel
            while (U.size() < FutureExactPredictions) {
                SparkFunOTOS.Pose2D vNow = Localizer.getVelocity();
                U.add(new SparkFunOTOS.Pose2D(vNow.x, vNow.y, vNow.h));
            }
            return U;
        }

        // dynamics: note the + velocity * dt, matching your C++ implementation
        private double dynamicFunc(double CurrentAxis, double VelocityOnAxis, double FrictionOnAxis) {
            return CurrentAxis + VelocityOnAxis * delta_t - FrictionOnAxis;
        }

        private SparkFunOTOS.Pose2D GenerateFuturePoint(SparkFunOTOS.Pose2D currState, SparkFunOTOS.Pose2D currVelocities) {
            return new SparkFunOTOS.Pose2D(
                    dynamicFunc(currState.x, currVelocities.x, Friction),
                    dynamicFunc(currState.y, currVelocities.y, Friction),
                    dynamicFunc(currState.h, currVelocities.h, Friction)
            );
        }

        private ArrayList<SparkFunOTOS.Pose2D> GenerateFuturePoints(SparkFunOTOS.Pose2D currPos, ArrayList<SparkFunOTOS.Pose2D> U) {
            ArrayList<SparkFunOTOS.Pose2D> X = new ArrayList<>();
            X.add(GenerateFuturePoint(currPos, U.get(0)));
            for (int i = 1; i < U.size(); i++) {
                X.add(GenerateFuturePoint(X.get(i - 1), U.get(i)));
            }
            return X;
        }

        private double CostFunctionAndDeriv(
                ArrayList<SparkFunOTOS.Pose2D> X,
                ArrayList<SparkFunOTOS.Pose2D> U,
                ArrayList<SparkFunOTOS.Pose2D> costDeriv) {

            double cost = 0.0;

            // initialize derivatives to zero and match U size
            costDeriv.clear();
            for (int i = 0; i < U.size(); i++) {
                costDeriv.add(new SparkFunOTOS.Pose2D(0, 0, 0));
            }

            // 1) Positional tracking cost (state-based)
            for (int i = 0; i < X.size(); i++) {
                SparkFunOTOS.Pose2D oneX = X.get(i);

                double dx = oneX.x - target.x;
                double dy = oneX.y - target.y;
                double dh = Localizer.getAngleDifference(oneX.h, target.h);

                cost += PositionalModifier * dx * dx;
                cost += PositionalModifier * dy * dy;
                cost += AngularModifier * dh * dh;

                // backprop to U (linear dynamics)
                for (int j = 0; j <= i; j++) {
                    SparkFunOTOS.Pose2D g = costDeriv.get(j);
                    double factor = (i - j + 1) * delta_t;
                    g.x += 2 * PositionalModifier * dx * factor;
                    g.y += 2 * PositionalModifier * dy * factor;
                    g.h += 2 * AngularModifier * dh * factor;
                }
            }

            // 3) Smoothness terms (between successive U)
            for (int i = 1; i < U.size(); i++) {
                SparkFunOTOS.Pose2D ui = U.get(i);
                SparkFunOTOS.Pose2D uj = U.get(i - 1);

                double dx = ui.x - uj.x;
                double dy = ui.y - uj.y;
                double dh = ui.h - uj.h;

                cost += VelocityPositionalModifier * dx * dx;
                cost += VelocityPositionalModifier * dy * dy;
                cost += VelocityAngularModifier * dh * dh;

                // gradient wrt ui
                SparkFunOTOS.Pose2D gi = costDeriv.get(i);
                gi.x += 2 * VelocityPositionalModifier * dx;
                gi.y += 2 * VelocityPositionalModifier * dy;
                gi.h += 2 * VelocityAngularModifier * dh;

                // gradient wrt uj
                SparkFunOTOS.Pose2D gj = costDeriv.get(i - 1);
                gj.x -= 2 * VelocityPositionalModifier * dx;
                gj.y -= 2 * VelocityPositionalModifier * dy;
                gj.h -= 2 * VelocityAngularModifier * dh;
            }

            return cost;
        }

        private double clamp(double val, double lo, double hi) {
            if (val < lo) return lo;
            if (val > hi) return hi;
            return val;
        }

        private void ApplyHardLimits(ArrayList<SparkFunOTOS.Pose2D> U) {
            // velocity limits (clamp to ±Max)
            for (int i = 0; i < U.size(); i++) {
                SparkFunOTOS.Pose2D u = U.get(i);
                u.x = clamp(u.x, -MaxVelocityX, MaxVelocityX);
                u.y = clamp(u.y, -MaxVelocityY, MaxVelocityY);
                u.h = clamp(u.h, -MaxVelocityH, MaxVelocityH);
                U.set(i, u);
            }

            // acceleration limits (first element relative to current velocity)
            SparkFunOTOS.Pose2D currentVel = Localizer.getVelocity();

            SparkFunOTOS.Pose2D u0 = U.get(0);
            u0.x = clamp(u0.x, currentVel.x - MaxAccelerationX, currentVel.x + MaxAccelerationX);
            u0.y = clamp(u0.y, currentVel.y - MaxAccelerationY, currentVel.y + MaxAccelerationY);
            u0.h = clamp(u0.h, currentVel.h - MaxAccelerationH, currentVel.h + MaxAccelerationH);
            U.set(0, u0);

            // rest relative to previous step
            for (int i = 1; i < U.size(); i++) {
                SparkFunOTOS.Pose2D prev = U.get(i - 1);
                SparkFunOTOS.Pose2D curr = U.get(i);
                curr.x = clamp(curr.x, prev.x - MaxAccelerationX, prev.x + MaxAccelerationX);
                curr.y = clamp(curr.y, prev.y - MaxAccelerationY, prev.y + MaxAccelerationY);
                curr.h = clamp(curr.h, prev.h - MaxAccelerationH, prev.h + MaxAccelerationH);
                U.set(i, curr);
            }
        }

        private void GeneratePrediction(SparkFunOTOS.Pose2D currPos) {

            ArrayList<SparkFunOTOS.Pose2D> U;
            U = GeneralGuessUWithWarm();

            mini = 1e9;

            for (int iter = 0; iter < MaxCorrectionInter; iter++) {

                // 1) simulate positions based on current U
                ArrayList<SparkFunOTOS.Pose2D> X = GenerateFuturePoints(currPos, U);

                // 2) compute cost and gradients
                ArrayList<SparkFunOTOS.Pose2D> costDeriv = new ArrayList<>();
                double J = CostFunctionAndDeriv(X, U, costDeriv);

                // 3) keep the best U sequence
                if (J < mini) {
                    mini = J;
                    bestU = U;
                }

                // 4) gradient descent update (C++ code updated by subtracting derivatives directly)
                for (int j = 0; j < U.size(); j++) {
                    SparkFunOTOS.Pose2D u = U.get(j);
                    SparkFunOTOS.Pose2D g = costDeriv.get(j);

                    U.set(j, new SparkFunOTOS.Pose2D(
                            u.x - g.x,
                            u.y - g.y,
                            u.h - g.h
                    ));
                }

                // enforce hard physical limits
                ApplyHardLimits(U);
            }
        }

        public SparkFunOTOS.Pose2D GetTargetVelocities() {
            GeneratePrediction(Localizer.getCurrentPosition());
            for (int i = 0; i < bestU.size(); i++)
                {
                    RobotInitializers.Dashtelemetry.addData(i + " bestU",bestU.get(i));
                }
            return bestU.get(0);
        }

        public void setGoalPos(SparkFunOTOS.Pose2D pos) {
            target = pos;
            FirstIteration = false; // reset warm-start
        }
    }
