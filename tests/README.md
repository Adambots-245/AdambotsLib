# Tests

Example test programs and validation code for AdambotsLib components.

## Overview

This folder contains standalone test programs that demonstrate and validate functionality of AdambotsLib utilities and components. These are **not** unit tests in the traditional sense, but rather example programs that can be run independently to verify behavior and serve as learning resources.

**Note:** These test files are kept outside the main library source code because they are examples and demonstrations, not production code that should be compiled into the library.

---

## Available Test Files

### StateMachineTest.java

**Purpose:** Comprehensive test and demonstration of the `StateMachine` utility class.

**What it tests:**
- Immediate transitions (position control mode)
- Conditional transitions (non-position control mode)
- State-property map usage
- Transition listeners
- Invalid transition handling
- Builder pattern API

**How to run:**

```bash
# Compile and run
javac -cp ".:path/to/AdambotsLib.jar" tests/StateMachineTest.java
java -cp ".:path/to/AdambotsLib.jar:tests" com.adambots.lib.utils.StateMachineTest
```

**What it demonstrates:**

1. **Mock Mechanism** - Simulates a robot mechanism with position control
2. **Immediate Transitions** - Position control where motor controller handles reaching target
3. **Conditional Transitions** - Non-position control where we wait for condition
4. **Transition Logging** - Shows all state transitions with timestamps
5. **Error Cases** - Demonstrates invalid transition handling

**Sample Output:**
```
=== StateMachine Test Suite ===

Test 1: Immediate Transition (Position Control)
[0ms] Transitioning: IDLE -> TARGET_A
[20ms] Transition complete: TARGET_A

Test 2: Conditional Transition (Non-Position Control)
[40ms] Transitioning: TARGET_A -> MOVING
[60ms] Waiting for condition...
[80ms] Waiting for condition...
[100ms] Condition met!
[120ms] Transition complete: MOVING

✓ All tests passed
```

---

## Usage

### As Learning Resources

These test files serve as:
- **Examples** of how to use AdambotsLib components
- **Reference implementations** showing best practices
- **Validation** that components work as expected

### For Development

When modifying AdambotsLib components:
1. Run relevant test files to verify changes don't break functionality
2. Update test files if API changes
3. Add new test cases for new features

### In Documentation

Test files are referenced in documentation:
- StateMachine.md references StateMachineTest.java for examples
- Shows real working code instead of just snippets

---

## Test File Structure

### Standard Format

Each test file should follow this structure:

```java
package com.adambots.lib.utils;

/**
 * Test program for [Component Name].
 *
 * Description of what this tests.
 */
public class ComponentTest {

    // Test data structures
    enum TestState { ... }
    record TestProperties(...) { }

    // Mock/simulation classes
    static class MockMechanism { ... }

    // Test methods
    public static void testFeature1() { ... }
    public static void testFeature2() { ... }

    // Main entry point
    public static void main(String[] args) {
        System.out.println("=== Component Test Suite ===\n");

        testFeature1();
        testFeature2();

        System.out.println("\n✓ All tests passed");
    }
}
```

---

## Adding New Test Files

When adding a new test file:

1. **Create the test file** in this `tests/` folder
2. **Follow the standard format** shown above
3. **Document what it tests** in the file header
4. **Update this README** with a new section describing the test
5. **Reference it in component documentation** if applicable

**Example:**
```bash
# Create new test file
touch tests/NewComponentTest.java

# Edit tests/README.md to add documentation
# Edit docs/utils/NewComponent.md to reference the test
```

---

## Differences from Unit Tests

**These are NOT unit tests:**
- Not part of automated test suite
- Run manually for verification
- Serve as examples and demonstrations
- Not using JUnit or other test frameworks

**Traditional unit tests** would go in `src/test/java/` and use:
- JUnit 5 framework
- Automated test runners
- Assertions and test fixtures
- CI/CD integration

**Why we use this approach:**
- Simpler for FRC teams to understand
- Easier to run standalone
- Serves dual purpose as documentation
- No dependency on test frameworks

---

## Best Practices

### 1. Keep Tests Simple

Tests should be easy to understand and run:
```java
// Good - simple and clear
public static void testImmediateTransition() {
    System.out.println("Test: Immediate Transition");
    sm.to(State.TARGET).executing(action).request();
    sm.periodic();
    assert sm.getCurrentState() == State.TARGET;
    System.out.println("✓ Passed\n");
}
```

### 2. Use Mock/Simulation Classes

Don't require actual robot hardware:
```java
static class MockMechanism {
    double position = 0.0;

    void setPosition(double pos) {
        this.position = pos;
    }

    boolean isAtTarget(double target) {
        return Math.abs(position - target) < 0.1;
    }
}
```

### 3. Print Clear Output

Make it obvious what's being tested and what passed:
```java
System.out.println("=== Test: Conditional Transitions ===");
System.out.println("Starting state: " + sm.getCurrentState());
// ... test code ...
System.out.println("✓ Test passed: State correctly transitioned\n");
```

### 4. Document Expected Behavior

Explain what should happen:
```java
/**
 * Test immediate transitions.
 *
 * Expected behavior:
 * - Transition should complete on next periodic() call
 * - No condition checking should occur
 * - State should change immediately
 */
public static void testImmediateTransition() { ... }
```

---

## Integration with Documentation

Test files should be referenced in component documentation:

**In StateMachine.md:**
```markdown
## Examples

For comprehensive examples including edge cases, see:
- [StateMachineTest.java](../../tests/StateMachineTest.java)
```

**In README files:**
```markdown
## Testing

Example test program available in `tests/StateMachineTest.java`
```

---

## Running Tests

### Prerequisites

1. AdambotsLib compiled and available
2. Java 17+ installed
3. Classpath includes AdambotsLib and WPILib dependencies

### Command Line

```bash
# From project root
javac -cp "lib/*:." tests/StateMachineTest.java
java -cp "lib/*:tests:." com.adambots.lib.utils.StateMachineTest
```

### In IDE

1. Open test file in your IDE
2. Ensure AdambotsLib is on classpath
3. Run the main() method

### Expected Output

Each test should print:
- Test name/description
- Current state/progress
- Success indicator (✓)
- Any errors or failures

---

## Future Enhancements

Potential improvements to test files:

1. **Add more test files** for other utilities as they're created
2. **Automate test running** with a simple shell script
3. **Add performance tests** for critical paths
4. **Create visual tests** for GUI components (if any)

---

## See Also

- [StateMachine Documentation](../docs/utils/StateMachine.md)
- [Utils Documentation](../docs/utils/README.md)
- [MAINTAINER.md](../MAINTAINER.md) - Development guidelines

---

**Last Updated:** 2026-01-14
