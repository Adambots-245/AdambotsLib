package com.adambots.lib.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * Generic state machine implementation for FRC robotics with builder pattern API.
 *
 * <p>Automatically supports both immediate transitions (position control) and conditional transitions
 * (non-position control) based on whether a condition is specified in the builder.
 *
 * <p>Example usage:
 * <pre>{@code
 * // Immediate transition (no condition)
 * stateMachine.to(ArmState.HIGH)
 *     .executing(props -> arm.setPosition(props.angle()))
 *     .request();
 *
 * // Conditional transition (waits for condition)
 * stateMachine.to(IntakeState.INTAKING)
 *     .executing(props -> intake.setSpeed(props.speed()))
 *     .when(intake::hasGamePiece)
 *     .request();
 * }</pre>
 *
 * @param <S> State enum type
 * @param <P> Properties type associated with each state
 */
public class StateMachine<S extends Enum<S>, P> {
    private S currentState;
    private S targetState;
    private P targetProperties;
    private final Consumer<String> logger;
    private BooleanSupplier atTargetCheck;
    private final List<BiConsumer<S, S>> transitionListeners;
    private final Map<String, String> invalidTransitions;
    private final Map<S, P> statePropertyMap;

    /**
     * Creates a new StateMachine.
     *
     * @param initialState The initial state
     * @param initialProperties Properties for the initial state
     * @param logger Optional logger for transition messages (can be null)
     */
    public StateMachine(S initialState, P initialProperties, Consumer<String> logger) {
        this.currentState = initialState;
        this.targetState = initialState;
        this.targetProperties = initialProperties;
        this.logger = logger != null ? logger : (msg -> {});
        this.transitionListeners = new ArrayList<>();
        this.invalidTransitions = new HashMap<>();
        this.statePropertyMap = null; // No property map in this constructor
    }

    /**
     * Creates a new StateMachine with automatic property lookup from a state-to-properties map.
     *
     * <p>This constructor allows states to automatically load their properties from a Map,
     * making the API even simpler: {@code stateMachine.to(State.HIGH).executing(action).request()}
     * without needing to manually pass properties.
     *
     * @param initialState The initial state
     * @param stateProperties Map from states to their properties
     * @param logger Optional logger for transition messages (can be null)
     */
    public StateMachine(S initialState, Map<S, P> stateProperties, Consumer<String> logger) {
        P initialProps = stateProperties.get(initialState);
        if (initialProps == null) {
            throw new IllegalArgumentException("No properties defined for initial state: " + initialState);
        }

        this.currentState = initialState;
        this.targetState = initialState;
        this.targetProperties = initialProps;
        this.logger = logger != null ? logger : (msg -> {});
        this.transitionListeners = new ArrayList<>();
        this.invalidTransitions = new HashMap<>();
        this.statePropertyMap = stateProperties;
    }

    /**
     * Gets the current state (actual state the system is in).
     *
     * @return Current state
     */
    public S getCurrentState() {
        return currentState;
    }

    /**
     * Gets the target state (commanded state).
     *
     * @return Target state
     */
    public S getTargetState() {
        return targetState;
    }

    /**
     * Gets the properties associated with the target state.
     *
     * @return Target properties
     */
    public P getTargetProperties() {
        return targetProperties;
    }

    /**
     * Checks if the state machine is currently transitioning.
     *
     * @return true if target state differs from current state
     */
    public boolean isTransitioning() {
        return targetState != currentState;
    }

    /**
     * Starts a new state transition using the builder pattern.
     *
     * <p>If a state-property map was provided in the constructor, properties are loaded automatically.
     * Otherwise, you must call {@link TransitionBuilder#withProperties(Object)} before {@code request()}.
     *
     * @param newState The desired state
     * @return TransitionBuilder for fluent API
     */
    public TransitionBuilder to(S newState) {
        return new TransitionBuilder(newState);
    }

    /**
     * Builder for fluent state transition API.
     *
     * <p>This builder allows you to configure a state transition step-by-step:
     * <ul>
     *   <li>{@link #withProperties(Object)} - Set properties (required if no state map)</li>
     *   <li>{@link #executing(Consumer)} - Set action to execute</li>
     *   <li>{@link #when(BooleanSupplier)} - Set condition for transition (optional)</li>
     *   <li>{@link #request()} - Execute the transition request</li>
     * </ul>
     *
     * <p>The mode (immediate vs conditional) is automatically detected:
     * <ul>
     *   <li>No {@code .when()} → Immediate transition (completes on next periodic())</li>
     *   <li>With {@code .when(condition)} → Conditional transition (waits for condition)</li>
     * </ul>
     */
    public class TransitionBuilder {
        private final S targetState;
        private P properties;
        private BooleanSupplier condition;
        private Consumer<P> action;

        private TransitionBuilder(S targetState) {
            this.targetState = targetState;

            // Auto-load properties from map if available
            if (statePropertyMap != null) {
                this.properties = statePropertyMap.get(targetState);
                if (this.properties == null) {
                    throw new IllegalStateException(
                        "No properties configured for state: " + targetState +
                        ". Check your state-property map."
                    );
                }
            }
        }

        /**
         * Sets the properties for this transition.
         *
         * <p>Required if the StateMachine was not created with a state-property map.
         * Optional if using state-property map (overrides the mapped properties).
         *
         * @param props Properties for the target state
         * @return this builder
         */
        public TransitionBuilder withProperties(P props) {
            this.properties = props;
            return this;
        }

        /**
         * Sets the action to execute when the transition is requested.
         *
         * <p>The action is executed immediately when {@link #request()} is called,
         * not when the state actually transitions. This allows you to command motors,
         * set outputs, etc. before waiting for the condition.
         *
         * @param action Action to execute (receives properties)
         * @return this builder
         */
        public TransitionBuilder executing(Consumer<P> action) {
            this.action = action;
            return this;
        }

        /**
         * Sets the condition for completing the transition (conditional mode).
         *
         * <p>If specified, the state machine will wait for this condition to be true
         * before completing the transition. The condition is checked in {@code periodic()}.
         *
         * <p>If not specified, the transition completes immediately on the next
         * {@code periodic()} call (immediate mode).
         *
         * @param condition Condition to check (e.g., {@code mechanism::atTarget})
         * @return this builder
         */
        public TransitionBuilder when(BooleanSupplier condition) {
            this.condition = condition;
            return this;
        }

        /**
         * Requests the state transition with the configured settings.
         *
         * <p>This method:
         * <ol>
         *   <li>Validates that properties are set</li>
         *   <li>Checks for invalid transitions</li>
         *   <li>Auto-detects mode (immediate if no condition, conditional otherwise)</li>
         *   <li>Executes the action (if provided)</li>
         *   <li>Sets up for state completion in {@code periodic()}</li>
         * </ol>
         *
         * @throws IllegalStateException if properties not set or transition is invalid
         */
        public void request() {
            // Validate properties
            if (properties == null) {
                throw new IllegalStateException(
                    "Properties required for transition to " + targetState +
                    ". Call withProperties() or use state-property map constructor."
                );
            }

            // Auto-detect mode: if no condition provided, create immediate transition
            BooleanSupplier finalCondition = (condition != null) ? condition : () -> true;

            // Delegate to internal method
            requestTransitionInternal(targetState, properties, finalCondition, action);
        }
    }

    /**
     * Internal method to request a state transition (called by TransitionBuilder).
     *
     * @param newState The desired state
     * @param properties Properties for the new state
     * @param atTarget Condition to check before completing transition
     * @param action Action to execute (e.g., command motors)
     * @throws IllegalStateException if the transition is marked as invalid
     */
    private void requestTransitionInternal(S newState, P properties, BooleanSupplier atTarget, Consumer<P> action) {
        if (newState != targetState) {
            // Check if this transition is invalid
            String transitionKey = currentState + "->" + newState;
            if (invalidTransitions.containsKey(transitionKey)) {
                throw new IllegalStateException(
                    "Invalid transition from " + currentState + " to " + newState +
                    ": " + invalidTransitions.get(transitionKey));
            }

            logger.accept("Transitioning from " + currentState + " to " + newState);

            this.targetState = newState;
            this.targetProperties = properties;
            this.atTargetCheck = atTarget;

            // Execute action immediately
            if (action != null) {
                action.accept(properties);
            }
            // State will transition in periodic() when atTarget returns true
        }
    }

    /**
     * Must be called periodically to update state transitions.
     *
     * <p>Checks the atTarget condition and completes the transition when the condition is met.
     * Includes fail-fast error checking for null conditions.
     */
    public void periodic() {
        if (targetState != currentState) {
            // Fail-fast: Null check should never happen, but protect against it
            if (atTargetCheck == null) {
                throw new IllegalStateException(
                    "Transition to " + targetState + " has null condition check. " +
                    "This indicates a bug in the state machine implementation."
                );
            }

            if (atTargetCheck.getAsBoolean()) {
                S previousState = currentState;
                currentState = targetState;
                logger.accept("Reached target state: " + targetState);
                notifyTransitionListeners(previousState, targetState);
            }
        }
    }

    /**
     * Adds a listener to be notified when state transitions complete.
     *
     * @param listener Callback that receives (fromState, toState)
     */
    public void addTransitionListener(BiConsumer<S, S> listener) {
        transitionListeners.add(listener);
    }

    /**
     * Marks a specific state transition as invalid.
     *
     * <p>If this transition is requested, an IllegalStateException will be thrown.
     * This is optional - by default, all transitions are allowed.
     *
     * @param from Source state
     * @param to Destination state
     * @param reason Explanation of why the transition is invalid
     */
    public void addInvalidTransition(S from, S to, String reason) {
        invalidTransitions.put(from + "->" + to, reason);
    }

    private void notifyTransitionListeners(S from, S to) {
        for (BiConsumer<S, S> listener : transitionListeners) {
            listener.accept(from, to);
        }
    }
}
