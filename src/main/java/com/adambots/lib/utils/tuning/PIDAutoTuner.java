package com.adambots.lib.utils.tuning;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.adambots.lib.utils.Dash;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * PID Auto-Tuner utility using the Relay Feedback Method (Ziegler-Nichols).
 *
 * <p>This utility automatically tunes PID gains by inducing controlled oscillations
 * and measuring the system response. It uses the classic Ziegler-Nichols formulas
 * to calculate optimal kP, kI, and kD values.
 *
 * <p><strong>Simple Usage:</strong>
 * <pre>{@code
 * // Position tuning - just pass method references!
 * Command tuneCmd = PIDAutoTuner.tunePosition(
 *     "Turret",                    // name for dashboard
 *     turret::getAngle,            // measurement method (DoubleSupplier)
 *     turret::setPercentOutput,    // output method (DoubleConsumer)
 *     -180, 180                    // safe range (degrees)
 * );
 *
 * // Bind to button
 * Buttons.XboxAButton.onTrue(tuneCmd);
 * }</pre>
 *
 * <p><strong>Velocity Tuning with Feedforward:</strong>
 * <pre>{@code
 * Command tuneCmd = PIDAutoTuner.tuneVelocity(
 *     "Shooter",
 *     shooter::getVelocityRPS,     // velocity measurement
 *     shooter::setVoltage,         // voltage output
 *     0, 100                       // velocity range (RPS)
 * );
 * }</pre>
 *
 * <p><strong>Getting Results:</strong>
 * <pre>{@code
 * // After tuning completes, get results:
 * TuningResult result = PIDAutoTuner.getLastResult("Turret");
 *
 * // Display string for copying to Constants
 * System.out.println(result.toCodeString());
 * // Output: kP = 0.05, kI = 0.001, kD = 0.02, kF = 0.0
 *
 * // Or apply directly to motor
 * result.applyTo(turretMotor, 0);
 * }</pre>
 *
 * <p><strong>Builder Pattern (Advanced):</strong>
 * <pre>{@code
 * Command tuneCmd = PIDAutoTuner.create("Elevator")
 *     .measureWith(elevator::getPosition)
 *     .controlWith(elevator::setPercentOutput)
 *     .range(0, 50)                    // rotations
 *     .maxOutput(0.3)                  // safety limit (30%)
 *     .withFeedforward(elevator::setVoltage)  // enable FF calculation
 *     .buildCommand();
 * }</pre>
 *
 * @see TuningResult
 */
public class PIDAutoTuner {

    /** Default oscillation amplitude (percent output) */
    private static final double DEFAULT_OSCILLATION_AMPLITUDE = 0.3;

    /** Default timeout for each phase in seconds */
    private static final double DEFAULT_PHASE_TIMEOUT = 10.0;

    /** Minimum oscillations required for accurate measurement */
    private static final int MIN_OSCILLATIONS = 3;

    /** Maximum oscillations to prevent runaway */
    private static final int MAX_OSCILLATIONS = 10;

    /** Step voltage for feedforward characterization */
    private static final double DEFAULT_STEP_VOLTAGE = 8.0;

    /** Cache of tuning results by name */
    private static final Map<String, TuningResult> resultsCache = new HashMap<>();

    // Dashboard entries for current tuning session
    private static final Map<String, GenericEntry> dashEntries = new HashMap<>();

    /**
     * Private constructor to prevent instantiation.
     */
    private PIDAutoTuner() {
        throw new UnsupportedOperationException("Utility class");
    }

    // ======================== STATIC FACTORY METHODS ========================

    /**
     * Creates a position tuning command.
     *
     * <p>This method uses relay feedback to induce oscillation around a setpoint
     * and calculates PID gains using Ziegler-Nichols formulas.
     *
     * @param name Name for dashboard display and result caching
     * @param measureFunc Supplier that returns current position (e.g., motor::getPosition)
     * @param outputFunc Consumer that sets motor output (-1 to 1) (e.g., motor::setPercentOutput)
     * @param minRange Minimum safe position value
     * @param maxRange Maximum safe position value
     * @return Command that performs tuning when run
     */
    public static Command tunePosition(String name, DoubleSupplier measureFunc,
                                       DoubleConsumer outputFunc, double minRange, double maxRange) {
        return create(name)
            .measureWith(measureFunc)
            .controlWith(outputFunc)
            .range(minRange, maxRange)
            .buildCommand();
    }

    /**
     * Creates a velocity tuning command with feedforward calculation.
     *
     * <p>This method tunes PID for velocity control and also calculates
     * feedforward values (kV, kS) using step response analysis.
     *
     * @param name Name for dashboard display and result caching
     * @param measureFunc Supplier that returns current velocity (e.g., motor::getVelocity)
     * @param voltageOutputFunc Consumer that sets motor voltage (e.g., motor::setVoltage)
     * @param minVelocity Minimum safe velocity value
     * @param maxVelocity Maximum safe velocity value
     * @return Command that performs tuning when run
     */
    public static Command tuneVelocity(String name, DoubleSupplier measureFunc,
                                       DoubleConsumer voltageOutputFunc, double minVelocity, double maxVelocity) {
        return create(name)
            .measureWith(measureFunc)
            .controlWith(voltageOutputFunc)
            .range(minVelocity, maxVelocity)
            .velocityMode()
            .withFeedforward(voltageOutputFunc)
            .buildCommand();
    }

    /**
     * Gets the last tuning result for a given name.
     *
     * @param name The name used during tuning
     * @return TuningResult if available, null if not yet tuned
     */
    public static TuningResult getLastResult(String name) {
        return resultsCache.get(name);
    }

    /**
     * Creates a new tuner builder for advanced configuration.
     *
     * @param name Name for dashboard display and result caching
     * @return TunerBuilder for fluent configuration
     */
    public static TunerBuilder create(String name) {
        return new TunerBuilder(name);
    }

    // ======================== ZIEGLER-NICHOLS CALCULATIONS ========================

    /**
     * Calculates PID gains using classic Ziegler-Nichols formulas.
     *
     * @param ku Ultimate gain (amplitude ratio that causes sustained oscillation)
     * @param tu Ultimate period (period of sustained oscillation in seconds)
     * @return TuningResult with calculated PID values
     */
    static TuningResult calculateZieglerNichols(double ku, double tu) {
        // Classic Ziegler-Nichols PID formulas
        double kP = 0.6 * ku;
        double kI = 1.2 * ku / tu;
        double kD = 0.075 * ku * tu;

        return TuningResult.pid(kP, kI, kD);
    }

    /**
     * Calculates feedforward values from step response data.
     *
     * @param steadyStateVelocity Steady-state velocity achieved
     * @param appliedVoltage Voltage that was applied
     * @param staticFrictionVoltage Minimum voltage to overcome friction
     * @return TuningResult with feedforward values
     */
    static TuningResult calculateFeedforward(double steadyStateVelocity, double appliedVoltage,
                                             double staticFrictionVoltage) {
        // kV = voltage per unit velocity
        double kV = (appliedVoltage - staticFrictionVoltage) / steadyStateVelocity;

        // kS = static friction compensation
        double kS = staticFrictionVoltage;

        return TuningResult.extended(0, 0, 0, kV, kS, 0, 0);
    }

    // ======================== DASHBOARD INTEGRATION ========================

    /**
     * Sets up dashboard entries for a tuning session.
     */
    private static void setupDashboard(String name) {
        Dash.useTab("PID Tuning");

        // Status display
        Dash.add(name + " Status", () -> "Ready");

        // Result values (will be updated after tuning)
        dashEntries.put(name + "_kP", Dash.addTunable(name + " kP", 0.0));
        dashEntries.put(name + "_kI", Dash.addTunable(name + " kI", 0.0));
        dashEntries.put(name + "_kD", Dash.addTunable(name + " kD", 0.0));
        dashEntries.put(name + "_kF", Dash.addTunable(name + " kF", 0.0));

        Dash.useDefaultTab();
    }

    /**
     * Updates dashboard with tuning results.
     */
    private static void updateDashboard(String name, TuningResult result) {
        GenericEntry kPEntry = dashEntries.get(name + "_kP");
        GenericEntry kIEntry = dashEntries.get(name + "_kI");
        GenericEntry kDEntry = dashEntries.get(name + "_kD");
        GenericEntry kFEntry = dashEntries.get(name + "_kF");

        if (kPEntry != null) kPEntry.setDouble(result.kP());
        if (kIEntry != null) kIEntry.setDouble(result.kI());
        if (kDEntry != null) kDEntry.setDouble(result.kD());
        if (kFEntry != null) kFEntry.setDouble(result.kF());

        // Add extended values if present
        if (result.hasExtendedFF()) {
            Dash.useTab("PID Tuning");
            dashEntries.computeIfAbsent(name + "_kV", k -> Dash.addTunable(name + " kV", result.kV()));
            dashEntries.computeIfAbsent(name + "_kS", k -> Dash.addTunable(name + " kS", result.kS()));
            dashEntries.computeIfAbsent(name + "_kA", k -> Dash.addTunable(name + " kA", result.kA()));
            dashEntries.computeIfAbsent(name + "_kG", k -> Dash.addTunable(name + " kG", result.kG()));

            GenericEntry kVEntry = dashEntries.get(name + "_kV");
            GenericEntry kSEntry = dashEntries.get(name + "_kS");
            GenericEntry kAEntry = dashEntries.get(name + "_kA");
            GenericEntry kGEntry = dashEntries.get(name + "_kG");

            if (kVEntry != null) kVEntry.setDouble(result.kV());
            if (kSEntry != null) kSEntry.setDouble(result.kS());
            if (kAEntry != null) kAEntry.setDouble(result.kA());
            if (kGEntry != null) kGEntry.setDouble(result.kG());

            Dash.useDefaultTab();
        }
    }

    // ======================== BUILDER CLASS ========================

    /**
     * Builder for configuring PID auto-tuning with advanced options.
     */
    @SuppressWarnings("unused") // velocityMode reserved for future algorithm selection
    public static class TunerBuilder {
        private final String name;
        private DoubleSupplier measureFunc;
        private DoubleConsumer outputFunc;
        private DoubleConsumer voltageOutputFunc;
        private double minRange = Double.NEGATIVE_INFINITY;
        private double maxRange = Double.POSITIVE_INFINITY;
        private double maxOutput = DEFAULT_OSCILLATION_AMPLITUDE;
        private double phaseTimeout = DEFAULT_PHASE_TIMEOUT;
        private boolean velocityMode = false;
        private boolean calculateFeedforward = false;

        /**
         * Creates a new TunerBuilder.
         *
         * @param name Name for dashboard and result caching
         */
        TunerBuilder(String name) {
            this.name = name;
        }

        /**
         * Sets the measurement function.
         *
         * @param measureFunc Supplier that returns current measurement
         * @return this builder for chaining
         */
        public TunerBuilder measureWith(DoubleSupplier measureFunc) {
            this.measureFunc = measureFunc;
            return this;
        }

        /**
         * Sets the output control function (percent output).
         *
         * @param outputFunc Consumer that accepts output value (-1 to 1)
         * @return this builder for chaining
         */
        public TunerBuilder controlWith(DoubleConsumer outputFunc) {
            this.outputFunc = outputFunc;
            return this;
        }

        /**
         * Sets the safe operating range.
         *
         * @param min Minimum safe value
         * @param max Maximum safe value
         * @return this builder for chaining
         */
        public TunerBuilder range(double min, double max) {
            this.minRange = min;
            this.maxRange = max;
            return this;
        }

        /**
         * Sets the maximum output during oscillation (safety limit).
         *
         * @param maxOutput Maximum output magnitude (0.0 to 1.0)
         * @return this builder for chaining
         */
        public TunerBuilder maxOutput(double maxOutput) {
            this.maxOutput = Math.abs(maxOutput);
            return this;
        }

        /**
         * Sets the timeout for each tuning phase.
         *
         * @param timeoutSeconds Timeout in seconds
         * @return this builder for chaining
         */
        public TunerBuilder timeout(double timeoutSeconds) {
            this.phaseTimeout = timeoutSeconds;
            return this;
        }

        /**
         * Enables velocity tuning mode.
         *
         * @return this builder for chaining
         */
        public TunerBuilder velocityMode() {
            this.velocityMode = true;
            return this;
        }

        /**
         * Enables feedforward calculation using step response.
         *
         * @param voltageOutputFunc Consumer that accepts voltage output
         * @return this builder for chaining
         */
        public TunerBuilder withFeedforward(DoubleConsumer voltageOutputFunc) {
            this.calculateFeedforward = true;
            this.voltageOutputFunc = voltageOutputFunc;
            return this;
        }

        /**
         * Builds and returns the tuning command.
         *
         * @return Command that performs the tuning when run
         */
        public Command buildCommand() {
            if (measureFunc == null) {
                throw new IllegalStateException("Measurement function not set. Call measureWith()");
            }
            if (outputFunc == null) {
                throw new IllegalStateException("Output function not set. Call controlWith()");
            }

            // Set up dashboard
            setupDashboard(name);

            return Commands.sequence(
                // Log start
                Commands.runOnce(() -> {
                    DriverStation.reportWarning("PID Auto-Tuner: Starting tuning for " + name, false);
                    System.out.println("\n========================================");
                    System.out.println("PID AUTO-TUNER: " + name);
                    System.out.println("========================================");
                }),

                // Main tuning command
                createTuningCommand(),

                // Log completion
                Commands.runOnce(() -> {
                    TuningResult result = resultsCache.get(name);
                    if (result != null) {
                        System.out.println("\n========================================");
                        System.out.println("TUNING COMPLETE: " + name);
                        System.out.println("----------------------------------------");
                        System.out.println(result.toCodeString());
                        System.out.println("========================================\n");
                        DriverStation.reportWarning("PID Auto-Tuner: Tuning complete for " + name +
                            " - kP=" + String.format("%.4f", result.kP()) +
                            ", kI=" + String.format("%.4f", result.kI()) +
                            ", kD=" + String.format("%.4f", result.kD()), false);
                    }
                })
            ).withName("PIDAutoTune_" + name);
        }

        /**
         * Creates the main tuning command sequence.
         */
        private Command createTuningCommand() {
            // Calculate setpoint as midpoint of range
            double setpoint = (minRange + maxRange) / 2.0;

            // State variables for oscillation measurement
            final double[] oscillationData = new double[4]; // [lastCrossTime, periodSum, crossCount, peakValue]
            final boolean[] crossingState = new boolean[1]; // [wasAboveSetpoint]
            final double[] startTime = new double[1];

            return Commands.sequence(
                // Phase 1: Move to setpoint region
                Commands.runOnce(() -> {
                    System.out.println("Phase 1: Moving to setpoint region...");
                    startTime[0] = Timer.getFPGATimestamp();
                }),

                Commands.run(() -> {
                    double current = measureFunc.getAsDouble();
                    double error = setpoint - current;

                    // Simple proportional control to reach setpoint
                    double output = Math.signum(error) * Math.min(maxOutput, Math.abs(error) * 0.1);
                    outputFunc.accept(output);
                }).until(() -> {
                    double current = measureFunc.getAsDouble();
                    double error = Math.abs(setpoint - current);
                    double range = maxRange - minRange;
                    // Within 5% of range or timeout
                    return error < range * 0.05 ||
                           (Timer.getFPGATimestamp() - startTime[0]) > phaseTimeout;
                }),

                // Phase 2: Relay feedback oscillation
                Commands.runOnce(() -> {
                    System.out.println("Phase 2: Starting relay feedback oscillation...");
                    oscillationData[0] = Timer.getFPGATimestamp(); // lastCrossTime
                    oscillationData[1] = 0; // periodSum
                    oscillationData[2] = 0; // crossCount
                    oscillationData[3] = 0; // peakValue
                    crossingState[0] = measureFunc.getAsDouble() > setpoint;
                    startTime[0] = Timer.getFPGATimestamp();
                }),

                Commands.run(() -> {
                    double current = measureFunc.getAsDouble();

                    // Safety check
                    if (current < minRange || current > maxRange) {
                        outputFunc.accept(0);
                        DriverStation.reportError("PID Auto-Tuner: Safety limit exceeded for " + name, false);
                        return;
                    }

                    // Bang-bang control (relay feedback)
                    boolean aboveSetpoint = current > setpoint;
                    double output = aboveSetpoint ? -maxOutput : maxOutput;
                    outputFunc.accept(output);

                    // Track peak amplitude
                    double amplitude = Math.abs(current - setpoint);
                    if (amplitude > oscillationData[3]) {
                        oscillationData[3] = amplitude;
                    }

                    // Detect zero crossings
                    if (aboveSetpoint != crossingState[0]) {
                        crossingState[0] = aboveSetpoint;
                        double now = Timer.getFPGATimestamp();

                        if (oscillationData[2] > 0) {
                            // Calculate period (time between same-direction crossings)
                            // We count half-periods, so multiply by 2 for full period
                            double halfPeriod = now - oscillationData[0];
                            oscillationData[1] += halfPeriod * 2;
                        }
                        oscillationData[0] = now;
                        oscillationData[2]++;

                        System.out.println("  Oscillation " + (int)oscillationData[2] +
                            " - Amplitude: " + String.format("%.3f", oscillationData[3]));
                    }
                }).until(() -> {
                    // Stop after enough oscillations or timeout
                    return oscillationData[2] >= MAX_OSCILLATIONS ||
                           (oscillationData[2] >= MIN_OSCILLATIONS * 2 &&
                            (Timer.getFPGATimestamp() - startTime[0]) > 3.0) ||
                           (Timer.getFPGATimestamp() - startTime[0]) > phaseTimeout;
                }),

                // Phase 3: Calculate results
                Commands.runOnce(() -> {
                    outputFunc.accept(0); // Stop motor

                    int crossings = (int) oscillationData[2];
                    if (crossings < MIN_OSCILLATIONS * 2) {
                        DriverStation.reportError("PID Auto-Tuner: Insufficient oscillations detected for " +
                            name + " (" + crossings + " crossings)", false);
                        resultsCache.put(name, TuningResult.pid(0, 0, 0));
                        return;
                    }

                    // Calculate Tu (ultimate period) - average full period
                    double tu = oscillationData[1] / (crossings / 2.0);

                    // Calculate Ku (ultimate gain)
                    // Ku = 4 * d / (pi * a) where d is relay amplitude, a is oscillation amplitude
                    double relayAmplitude = maxOutput;
                    double oscillationAmplitude = oscillationData[3];
                    double ku = (4.0 * relayAmplitude) / (Math.PI * oscillationAmplitude);

                    System.out.println("\nOscillation Analysis:");
                    System.out.println("  Ultimate Period (Tu): " + String.format("%.3f", tu) + " seconds");
                    System.out.println("  Ultimate Gain (Ku): " + String.format("%.4f", ku));
                    System.out.println("  Oscillation Amplitude: " + String.format("%.3f", oscillationAmplitude));

                    // Calculate PID using Ziegler-Nichols
                    TuningResult result = calculateZieglerNichols(ku, tu);
                    resultsCache.put(name, result);
                    updateDashboard(name, result);
                }),

                // Phase 4: Feedforward characterization (optional)
                calculateFeedforward ?
                    createFeedforwardCommand(setpoint, startTime, oscillationData) :
                    Commands.none()
            ).finallyDo(() -> {
                // Ensure motor is stopped on abort
                outputFunc.accept(0);
            });
        }

        /**
         * Creates the feedforward characterization command.
         */
        private Command createFeedforwardCommand(double setpoint, double[] startTime, double[] ffData) {
            if (voltageOutputFunc == null) {
                return Commands.none();
            }

            return Commands.sequence(
                Commands.runOnce(() -> {
                    System.out.println("\nPhase 4: Measuring feedforward (step response)...");
                    startTime[0] = Timer.getFPGATimestamp();
                    ffData[0] = 0; // steadyStateVelocity
                    ffData[1] = 0; // staticFrictionVoltage
                    ffData[2] = 0; // velocitySum
                    ffData[3] = 0; // sampleCount
                }),

                // Find static friction threshold
                Commands.run(() -> {
                    double voltage = (Timer.getFPGATimestamp() - startTime[0]) * 2.0; // Ramp up slowly
                    voltageOutputFunc.accept(voltage);

                    double velocity = Math.abs(measureFunc.getAsDouble());
                    if (velocity > 0.1 && ffData[1] == 0) {
                        ffData[1] = voltage; // Record voltage that overcame friction
                        System.out.println("  Static friction voltage (kS): " + String.format("%.2f", voltage) + "V");
                    }
                }).until(() -> {
                    return ffData[1] > 0 || (Timer.getFPGATimestamp() - startTime[0]) > 3.0;
                }),

                // Apply step voltage and measure steady state
                Commands.runOnce(() -> {
                    startTime[0] = Timer.getFPGATimestamp();
                    voltageOutputFunc.accept(DEFAULT_STEP_VOLTAGE);
                }),

                Commands.run(() -> {
                    // Wait for steady state, then sample
                    if (Timer.getFPGATimestamp() - startTime[0] > 1.0) {
                        double velocity = Math.abs(measureFunc.getAsDouble());
                        ffData[2] += velocity;
                        ffData[3]++;
                    }
                }).until(() -> {
                    return ffData[3] > 20 || (Timer.getFPGATimestamp() - startTime[0]) > 3.0;
                }),

                // Calculate feedforward
                Commands.runOnce(() -> {
                    voltageOutputFunc.accept(0);

                    if (ffData[3] > 0) {
                        double steadyStateVelocity = ffData[2] / ffData[3];
                        double staticFriction = ffData[1];

                        System.out.println("  Steady-state velocity: " + String.format("%.2f", steadyStateVelocity));
                        System.out.println("  Applied voltage: " + DEFAULT_STEP_VOLTAGE + "V");

                        TuningResult ffResult = calculateFeedforward(steadyStateVelocity,
                            DEFAULT_STEP_VOLTAGE, staticFriction);

                        // Combine with existing PID result
                        TuningResult pidResult = resultsCache.get(name);
                        if (pidResult != null) {
                            TuningResult combined = TuningResult.extended(
                                pidResult.kP(), pidResult.kI(), pidResult.kD(),
                                ffResult.kV(), ffResult.kS(), 0, 0
                            );
                            resultsCache.put(name, combined);
                            updateDashboard(name, combined);
                        }
                    }
                })
            );
        }
    }
}
