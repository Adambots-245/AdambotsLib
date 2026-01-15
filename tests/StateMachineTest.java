package com.adambots.lib.utils;

import java.util.ArrayList;
import java.util.List;

/**
 * Test program for the StateMachine class with new builder pattern API.
 *
 * This tests both immediate transitions (position control mode) and
 * conditional transitions (non-position control mode).
 */
public class StateMachineTest {

    // Test states
    enum TestState {
        IDLE(new TestProperties(0.0, "Idle state")),
        MOVING(new TestProperties(50.0, "Moving state")),
        TARGET_A(new TestProperties(100.0, "Target A")),
        TARGET_B(new TestProperties(200.0, "Target B")),
        ERROR(new TestProperties(-1.0, "Error state"));

        public final TestProperties properties;

        TestState(TestProperties properties) {
            this.properties = properties;
        }
    }

    // Test properties record
    record TestProperties(double position, String description) {}

    // Mock mechanism state
    static class MockMechanism {
        double currentPosition = 0.0;
        double targetPosition = 0.0;
        double speed = 10.0; // Units per periodic cycle

        void setTarget(double target) {
            this.targetPosition = target;
        }

        void simulateMovement() {
            double error = targetPosition - currentPosition;
            if (Math.abs(error) < speed) {
                currentPosition = targetPosition;
            } else {
                currentPosition += Math.signum(error) * speed;
            }
        }

        boolean atTarget() {
            return Math.abs(currentPosition - targetPosition) < 0.1;
        }

        @Override
        public String toString() {
            return String.format("Position: %.1f, Target: %.1f, AtTarget: %s",
                currentPosition, targetPosition, atTarget());
        }
    }

    public static void main(String[] args) {
        System.out.println("=== StateMachine Test Suite (NEW API) ===\n");

        testImmediateTransitions();
        System.out.println();
        testConditionalTransitions();
        System.out.println();
        testTransitionListeners();
        System.out.println();
        testInvalidTransitions();
        System.out.println();
        testIsTransitioning();

        System.out.println("\n=== All Tests Complete ===");
    }

    /**
     * Test 1: Immediate transitions (position control mode)
     * When no .when() clause is provided, transitions happen on first periodic() call.
     */
    static void testImmediateTransitions() {
        System.out.println("Test 1: Immediate Transitions (Position Control Mode)");
        System.out.println("-----------------------------------------------------");

        List<String> log = new ArrayList<>();
        StateMachine<TestState, TestProperties> sm = new StateMachine<>(
            TestState.IDLE,
            TestState.IDLE.properties,
            log::add
        );

        MockMechanism mechanism = new MockMechanism();

        System.out.println("Initial state: " + sm.getCurrentState());
        System.out.println("Initial target: " + sm.getTargetState());

        // NEW API: Request immediate transition (no .when() clause)
        sm.to(TestState.TARGET_A)
            .withProperties(TestState.TARGET_A.properties)
            .executing(props -> {
                mechanism.setTarget(props.position());
                System.out.println("  Action executed: Set target to " + props.position());
            })
            .request();

        System.out.println("\nAfter request (before periodic):");
        System.out.println("  Current state: " + sm.getCurrentState());
        System.out.println("  Target state: " + sm.getTargetState());
        System.out.println("  Expected: Current=IDLE, Target=TARGET_A");

        if (sm.getCurrentState() == TestState.IDLE &&
            sm.getTargetState() == TestState.TARGET_A) {
            System.out.println("✓ State has not transitioned yet (waiting for periodic)");
        } else {
            System.out.println("✗ FAIL: State should not have transitioned before periodic()");
        }

        // Call periodic once - should transition immediately since no condition specified
        sm.periodic();

        System.out.println("\nAfter periodic():");
        System.out.println("  Current state: " + sm.getCurrentState());
        System.out.println("  Target state: " + sm.getTargetState());
        System.out.println("  Expected: Both should be TARGET_A");

        // Verify
        if (sm.getCurrentState() == TestState.TARGET_A &&
            sm.getTargetState() == TestState.TARGET_A) {
            System.out.println("✓ PASS: Immediate transition worked correctly (on first periodic)");
        } else {
            System.out.println("✗ FAIL: State did not transition in periodic()");
        }

        // Print log
        System.out.println("\nLog messages:");
        log.forEach(msg -> System.out.println("  - " + msg));
    }

    /**
     * Test 2: Conditional transitions (non-position control mode)
     * When .when() clause is provided, transition waits until condition is met.
     */
    static void testConditionalTransitions() {
        System.out.println("Test 2: Conditional Transitions (Non-Position Control Mode)");
        System.out.println("-----------------------------------------------------------");

        List<String> log = new ArrayList<>();
        StateMachine<TestState, TestProperties> sm = new StateMachine<>(
            TestState.IDLE,
            TestState.IDLE.properties,
            log::add
        );

        MockMechanism mechanism = new MockMechanism();

        System.out.println("Initial state: " + sm.getCurrentState());

        // NEW API: Request conditional transition (with .when() clause)
        sm.to(TestState.TARGET_A)
            .withProperties(TestState.TARGET_A.properties)
            .executing(props -> {
                mechanism.setTarget(props.position());
                System.out.println("  Action executed: Set target to " + props.position());
            })
            .when(mechanism::atTarget)  // Wait for this condition
            .request();

        System.out.println("\nAfter request (before periodic):");
        System.out.println("  Current state: " + sm.getCurrentState());
        System.out.println("  Target state: " + sm.getTargetState());
        System.out.println("  Expected: Current=IDLE, Target=TARGET_A");

        if (sm.getCurrentState() == TestState.IDLE &&
            sm.getTargetState() == TestState.TARGET_A) {
            System.out.println("✓ PASS: State did not transition immediately");
        } else {
            System.out.println("✗ FAIL: State should not have transitioned yet");
        }

        // Simulate periodic cycles until target reached
        System.out.println("\nSimulating periodic cycles:");
        int cycle = 0;
        while (!mechanism.atTarget() && cycle < 20) {
            mechanism.simulateMovement();
            sm.periodic();
            cycle++;
            System.out.printf("  Cycle %2d: %s | State: %s\n",
                cycle, mechanism, sm.getCurrentState());
        }

        System.out.println("\nFinal state:");
        System.out.println("  Current state: " + sm.getCurrentState());
        System.out.println("  Target state: " + sm.getTargetState());
        System.out.println("  Mechanism: " + mechanism);

        if (sm.getCurrentState() == TestState.TARGET_A &&
            mechanism.atTarget()) {
            System.out.println("✓ PASS: Conditional transition completed when condition was met");
        } else {
            System.out.println("✗ FAIL: Transition did not complete correctly");
        }

        // Print log
        System.out.println("\nLog messages:");
        log.forEach(msg -> System.out.println("  - " + msg));
    }

    /**
     * Test 3: Transition listeners
     */
    static void testTransitionListeners() {
        System.out.println("Test 3: Transition Listeners");
        System.out.println("----------------------------");

        List<String> transitions = new ArrayList<>();
        StateMachine<TestState, TestProperties> sm = new StateMachine<>(
            TestState.IDLE,
            TestState.IDLE.properties,
            null
        );

        // Add transition listener
        sm.addTransitionListener((from, to) -> {
            String transition = from + " -> " + to;
            transitions.add(transition);
            System.out.println("  Listener notified: " + transition);
        });

        // Test immediate transition (no .when())
        System.out.println("\nTriggering immediate transition:");
        sm.to(TestState.TARGET_A)
            .withProperties(TestState.TARGET_A.properties)
            .request();  // Immediate - no condition
        sm.periodic(); // Complete the transition

        // Test conditional transition (with .when())
        System.out.println("\nTriggering conditional transition:");
        sm.to(TestState.TARGET_B)
            .withProperties(TestState.TARGET_B.properties)
            .when(() -> true)  // Condition immediately true
            .request();
        sm.periodic(); // Should trigger the transition

        System.out.println("\nRecorded transitions:");
        transitions.forEach(t -> System.out.println("  - " + t));

        if (transitions.size() == 2) {
            System.out.println("✓ PASS: All transitions were captured by listener");
        } else {
            System.out.println("✗ FAIL: Expected 2 transitions, got " + transitions.size());
        }
    }

    /**
     * Test 4: Invalid transition validation
     */
    static void testInvalidTransitions() {
        System.out.println("Test 4: Invalid Transition Validation");
        System.out.println("-------------------------------------");

        StateMachine<TestState, TestProperties> sm = new StateMachine<>(
            TestState.IDLE,
            TestState.IDLE.properties,
            null
        );

        // Mark ERROR as invalid from any state
        sm.addInvalidTransition(TestState.IDLE, TestState.ERROR, "Cannot transition to ERROR from IDLE");
        System.out.println("Configured: IDLE -> ERROR as invalid");

        // Try valid transition first
        System.out.println("\nAttempting valid transition (IDLE -> TARGET_A):");
        try {
            sm.to(TestState.TARGET_A)
                .withProperties(TestState.TARGET_A.properties)
                .request();
            sm.periodic();
            System.out.println("✓ Valid transition succeeded");
        } catch (IllegalStateException e) {
            System.out.println("✗ Valid transition should not throw: " + e.getMessage());
        }

        // Move back to IDLE
        sm.to(TestState.IDLE)
            .withProperties(TestState.IDLE.properties)
            .request();
        sm.periodic();

        // Try invalid transition
        System.out.println("\nAttempting invalid transition (IDLE -> ERROR):");
        try {
            sm.to(TestState.ERROR)
                .withProperties(TestState.ERROR.properties)
                .request();
            System.out.println("✗ FAIL: Invalid transition should have thrown exception");
        } catch (IllegalStateException e) {
            System.out.println("✓ PASS: Invalid transition rejected");
            System.out.println("  Exception message: " + e.getMessage());
        }
    }

    /**
     * Test 5: isTransitioning() method
     */
    static void testIsTransitioning() {
        System.out.println("Test 5: isTransitioning() Method");
        System.out.println("--------------------------------");

        StateMachine<TestState, TestProperties> sm = new StateMachine<>(
            TestState.IDLE,
            TestState.IDLE.properties,
            null
        );

        MockMechanism mechanism = new MockMechanism();

        System.out.println("Initial: " + sm.isTransitioning());
        if (!sm.isTransitioning()) {
            System.out.println("✓ Not transitioning initially");
        }

        // Start conditional transition
        sm.to(TestState.TARGET_A)
            .withProperties(TestState.TARGET_A.properties)
            .executing(props -> mechanism.setTarget(props.position()))
            .when(mechanism::atTarget)
            .request();

        System.out.println("After requesting transition: " + sm.isTransitioning());
        if (sm.isTransitioning()) {
            System.out.println("✓ isTransitioning() returns true during transition");
        }

        // Complete transition
        while (!mechanism.atTarget()) {
            mechanism.simulateMovement();
            sm.periodic();
        }

        System.out.println("After reaching target: " + sm.isTransitioning());
        if (!sm.isTransitioning()) {
            System.out.println("✓ PASS: isTransitioning() returns false when complete");
        } else {
            System.out.println("✗ FAIL: Should not be transitioning");
        }
    }
}
