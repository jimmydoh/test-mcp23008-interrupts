"""
Comprehensive test suite for MCP23008 interrupt functionality on Raspberry Pi Pico.
"""

import time
import random
import board
import busio
import digitalio
from adafruit_mcp230xx.mcp23008 import MCP23008

print("Initializing hardware...")

i2c = busio.I2C(board.GP5, board.GP4)
mcp = MCP23008(i2c, address=0x20)

# Setup output triggers on Pico
TRIGGER_PINS = [
    board.GP6, board.GP7, board.GP8, board.GP9,
    board.GP10, board.GP11, board.GP12, board.GP13
]
pico_outputs = []
for pin in TRIGGER_PINS:
    p = digitalio.DigitalInOut(pin)
    p.direction = digitalio.Direction.OUTPUT
    p.value = False
    pico_outputs.append(p)

# Setup interrupt listener on Pico
int_listener = digitalio.DigitalInOut(board.GP15)
int_listener.direction = digitalio.Direction.INPUT
int_listener.pull = digitalio.Pull.UP

# Setup MCP23008 pins as inputs
for pin_num in range(8):
    mcp_pin = mcp.get_pin(pin_num)
    mcp_pin.direction = digitalio.Direction.INPUT

# Setup Initial Interrupt config on MCP23008
mcp.interrupt_enable = 0xFF
mcp.interrupt_configuration = 0x00
mcp.io_control = 0x04 # Open-drain INT pin
mcp.clear_ints()


def reset():
    """Reset all pins and clear interrupts"""
    for p in pico_outputs:
        p.value = False
    mcp.clear_ints()
    time.sleep(0.2)


def reset_state():
        """
        Resets all config values to a known state and calls reset()
        """
        mcp.interrupt_enable = 0x00
        mcp.interrupt_configuration = 0x00
        mcp.default_value = 0x00
        mcp.io_control = 0x04
        reset()


def run_slow_test():
    """
    Tests each pin individually for rising and falling edges, verifying:
    - INTF and INTCAP values match expected results
    - Auto-clear behavior of the interrupt after reading INTCAP
    - Presence of the hardware interrupt signal on the Pico's INT listener pin
    """
    print("\n--- Starting Slow Cycle Test ---")
    errors = 0

    reset()

    for i in range(8):
        print(f"\nTesting Pin {i}...")

        # --- Test Rising Edge ---
        pico_outputs[i].value = True
        time.sleep(0.2)

        if int_listener.value == False:
            flags = mcp.int_flag
            caps = mcp.int_cap

            if i in flags and caps[i] == 1:
                print(f"  [PASS] Rising edge detected. INTF: {flags}, INTCAP pin {i}: {caps[i]}")
            else:
                print(f"  [FAIL] Flag or Cap mismatch! Flags: {flags}, Cap: {caps[i]}")
                errors += 1

            time.sleep(0.2)

            if int_listener.value == False:
                print(f"  [FAIL] Reading INTCAP did not clear the hardware interrupt!")
                errors += 1
                mcp.clear_ints()

        else:
            print(f"  [FAIL] No hardware interrupt signal detected on rising edge!")
            errors += 1

        time.sleep(0.2)

        # --- Test Falling Edge ---
        pico_outputs[i].value = False
        time.sleep(0.2)

        if int_listener.value == False:
            flags = mcp.int_flag
            caps = mcp.int_cap

            if i in flags and caps[i] == 0:
                print(f"  [PASS] Falling edge detected. INTF: {flags}, INTCAP pin {i}: {caps[i]}")
            else:
                print(f"  [FAIL] Flag or Cap mismatch! Flags: {flags}, Cap: {caps[i]}")
                errors += 1
        else:
            print(f"  [FAIL] No hardware interrupt signal detected on falling edge!")
            errors += 1

        time.sleep(0.2)

    reset()
    return errors


def run_multi_pin_test():
    """
    Tests firing multiple pins in rapid succession to verify:
    - Only the first event triggers the interrupt and is captured in INTF and INTCAP
    - Subsequent events do not trigger additional interrupts until the first one is cleared
    - Live GPIO state can be read correctly even when an interrupt is active
    """
    print("\n--- Starting Multi-Pin Test ---")
    errors = 0

    reset()

    print("  Firing pins 2, 3, and 4 in rapid succession...")
    # Fire 3 pins fast. The MCP will trigger the interrupt on the first one (pin 2).
    pico_outputs[2].value = True
    pico_outputs[3].value = True
    pico_outputs[4].value = True
    time.sleep(0.2)

    if int_listener.value == False:
        flags = mcp.int_flag
        caps = mcp.int_cap # Captures state at moment of Pin 2 changing, clears INT

        # INTF and INTCAP should reflect the FIRST event (Pin 2 going high)
        if 2 in flags and caps[2] == 1:
            print(f"  [PASS] INTF and INTCAP correctly captured the first event (Pin 2).")
        else:
            print(f"  [FAIL] Expected Pin 2 to be the captor. Flags: {flags}")
            errors += 1

        # Now let's read the CURRENT live state of the GPIOs using get_pin().value
        # This proves the other pins actually changed and can be read normally.
        pin2_live = mcp.get_pin(2).value
        pin3_live = mcp.get_pin(3).value
        pin4_live = mcp.get_pin(4).value

        if pin2_live and pin3_live and pin4_live:
            print("  [PASS] Live GPIO read correctly shows pins 2, 3, and 4 are currently High.")
        else:
            print("  [FAIL] Live GPIO read did not show all 3 pins high!")
            errors += 1
    else:
        print("  [FAIL] Hardware interrupt did not fire for multi-pin test!")
        errors += 1

    reset()
    return errors


def run_stress_test(iterations=500):
    """
    Runs a rapid stress test by randomly toggling pins for a specified number of iterations, verifying:
    - Each interrupt is correctly triggered and captured in INTF and INTCAP
    - No missed interrupts or incorrect captures occur under rapid changes
    - The system remains responsive and accurate throughout the test duration
    """
    errors = 0
    states = [False] * 8 
    
    # 1. Explicitly guarantee a clean configuration state
    mcp.interrupt_enable = 0xFF 
    mcp.interrupt_configuration = 0x00 
    mcp.default_value = 0x00
    mcp.io_control = 0x04 # Open-drain
    
    # 2. Reset physical pins
    for p in pico_outputs: 
        p.value = False
        
    time.sleep(0.05) # Allow electrical lines to physically settle
    
    # 3. Firmly lock in the baseline latch
    _ = mcp.gpio 
    mcp.clear_ints()
    
    start_time = time.monotonic()
    
    for count in range(iterations):
        iter_fail = False
        target_pin = random.randint(0, 7)
        new_state = not states[target_pin]
        states[target_pin] = new_state
        
        # Trigger the change
        pico_outputs[target_pin].value = new_state
        
        # Wait 3ms for the MCP23008 glitch filter and logic to assert INT
        time.sleep(0.003) 
        
        if int_listener.value == True:
            errors += 1
            # Diagnostic: What does the chip actually see on its pins right now?
            live_gpio = mcp.gpio
            live_bit = (live_gpio >> target_pin) & 1
            iter_fail = True
            print(f"  [FAIL] Iteration {count}: INT line did not drop! Pin {target_pin} -> {new_state}. Chip sees pin as: {live_bit}")
        
        # Read the snapshot data
        flags = mcp.int_flag
        caps = mcp.int_cap 
        
        # READ GPIO! This ensures the "previous state" baseline is fully 
        # updated to the current state of all pins for the next loop.
        _ = mcp.gpio 
        
        # VERY IMPORTANT: Give the silicon a tiny breathing room to reset 
        # its internal interrupt latches before we immediately toggle the next pin.
        time.sleep(0.002)
        
        # Verify accuracy
        if target_pin not in flags:
            errors += 1
            iter_fail = True
            print(f"  [FAIL] Iteration {count}: Flag array {flags} missing expected pin {target_pin}")
            
        expected_cap_val = 1 if new_state else 0
        if target_pin in flags and caps[target_pin] != expected_cap_val:
            errors += 1
            iter_fail = True
            print(f"  [FAIL] Iteration {count}: INTCAP {caps[target_pin]} != Expected {expected_cap_val}")

        #if not iter_fail:
        #    print(f"  [PASS] Iteration {count}: Pin {target_pin} -> {new_state}, Flags: {flags}, Caps: {caps[target_pin]}")

        if not iter_fail and count % 50 == 0:
            print(f"  [INFO] Completed {count} iterations with {errors} errors so far.")

    elapsed = time.monotonic() - start_time
    print(f"Stress test complete in {elapsed:.2f} seconds.")
    return errors


def run_config_test():
    """
    Tests various interrupt configuration settings on the MCP23008 to verify:
    - Global interrupt enable/disable functionality
    - Individual pin interrupt enable/disable functionality
    - INTCON compare against DEFVAL behavior
    - IOCON drive mode effects on the INT pin signal
        (open-drain vs push-pull, active-high vs active-low)
    """
    print("\n--- Starting Configuration Test ---")
    errors = 0

    # Test 1 - Global Disable
    reset_state()
    print("\n  Testing global interrupt disable...")
    mcp.interrupt_enable = 0x00
    pico_outputs[0].value = True
    time.sleep(0.05)

    if int_listener.value == False:
        print("  [FAIL] Interrupt triggered when it should be disabled!")
        errors += 1
    else:
        print("  [PASS] No interrupt triggered when globally disabled.")

    # --- Test 2 - Individual Pin Enable ---
    reset_state()
    print("\n  Testing individual pin enable (Pin 1 only)...")
    mcp.interrupt_enable = 0x02 # Enable ONLY pin 1 (0b00000010)
    mcp.io_control = 0x04       # Open-drain (Active Low)

    # Toggle Pin 0 (Should NOT fire)
    pico_outputs[0].value = True
    time.sleep(0.05)
    if int_listener.value == False:
        print("  [FAIL] Pin 0 triggered an interrupt but was not enabled!")
        errors += 1
    else:
        print("  [PASS] Pin 0 safely ignored.")

    # Toggle Pin 1 (SHOULD fire)
    pico_outputs[1].value = True
    time.sleep(0.05)
    if int_listener.value == False and 1 in mcp.int_flag:
        print("  [PASS] Pin 1 successfully triggered the interrupt.")
    else:
        print("  [FAIL] Pin 1 failed to trigger the interrupt!")
        errors += 1

    # --- Test 3 - INTCON Compare Against DEFVAL ---
    reset_state()
    print("\n  Testing INTCON and DEFVAL (Compare to Default)...")
    mcp.interrupt_enable = 0x01        # Enable Pin 0
    mcp.default_value = 0x00           # Pin 0 default is LOW
    mcp.interrupt_configuration = 0x01 # Pin 0 triggers when it DIFFERS from DEFVAL
    mcp.io_control = 0x04
    
    pico_outputs[0].value = True # Change to HIGH (differs from DEFVAL)
    time.sleep(0.05)
    if int_listener.value == False and 0 in mcp.int_flag:
        print("  [PASS] Interrupt fired when pin differed from DEFVAL.")
    else:
        print("  [FAIL] Failed to fire on DEFVAL difference.")
        errors += 1

    # IMPORTANT: Do NOT clear the interrupt here! 
    # If we clear it while the pin is still HIGH, the MCP23008 will instantly
    # re-trigger it because the pin STILL differs from DEFVAL.
    
    # First, return the pin to LOW (so it matches DEFVAL again).
    # This removes the physical condition causing the interrupt.
    pico_outputs[0].value = False 
    time.sleep(0.05)
    
    # NOW clear the interrupt that was generated from the initial change
    mcp.clear_ints()
    time.sleep(0.05)
    
    # Verify that returning to DEFVAL did not generate a *new* interrupt
    if int_listener.value == False:
         print("  [FAIL] Interrupt fired when returning to DEFVAL!")
         errors += 1
    else:
         print("  [PASS] No new interrupt triggered when pin matches DEFVAL.")

    # --- Test 4 - IOCON Push-Pull Active-High ---
    reset_state()
    print("\n  Testing IOCON Drive Modes (Push-Pull Active-High)...")
    mcp.interrupt_enable = 0x01

    # Bit 1 = INTPOL (1=Active High). Bit 2 = ODR (0=Push-Pull) -> 0x02
    mcp.io_control = 0x02
    time.sleep(0.05)

    # In Active-High Push-Pull, the idle state is actively driven LOW by the MCP.
    if int_listener.value == True:
        print("  [FAIL] INT pin is not being driven LOW during idle state!")
        errors += 1
    else:
        print("  [PASS] Idle state correctly driven LOW.")

    pico_outputs[0].value = True
    time.sleep(0.05)

    # When triggered, the MCP should actively drive the INT pin HIGH
    if int_listener.value == True and 0 in mcp.int_flag:
        print("  [PASS] INT pin correctly driven HIGH on interrupt.")
    else:
        print("  [FAIL] INT pin was not driven HIGH on interrupt!")
        errors += 1

    # --- Test 5 - IOCON Push-Pull Active-Low ---
    reset_state()
    print("\n  Testing IOCON Drive Modes (Push-Pull Active-Low)...")
    mcp.interrupt_enable = 0x01

    # Bit 1 = INTPOL (0=Active Low). Bit 2 = ODR (0=Push-Pull) -> 0x00
    mcp.io_control = 0x00
    time.sleep(0.05)

    # In Active-Low Push-Pull, the idle state is actively driven HIGH.
    # (This matches the Pico's pull-up, but we verify it's at least logic HIGH).
    if int_listener.value == False:
        print("  [FAIL] INT pin is not HIGH during idle state!")
        errors += 1
    else:
        print("  [PASS] Idle state is HIGH.")

    pico_outputs[0].value = True
    time.sleep(0.05)

    # When triggered, the MCP should actively drive the INT pin LOW
    if int_listener.value == False and 0 in mcp.int_flag:
        print("  [PASS] INT pin correctly driven LOW on interrupt.")
    else:
        print("  [FAIL] INT pin was not driven LOW on interrupt!")
        errors += 1

    # Clean up and revert back to Open-Drain for the rest of the script
    reset_state()
    mcp.interrupt_enable = 0xFF

    reset_state()
    return errors


# Main execution flow
try:
    slow_errors = run_slow_test()
    multi_errors = run_multi_pin_test()
    config_errors = run_config_test()

    total_pre_errors = slow_errors + multi_errors + config_errors

    if total_pre_errors > 0:
        print(f"\nPre-tests finished with {total_pre_errors} errors. Aborting stress test.")
    else:
        stress_errors = run_stress_test(5000)

        if stress_errors == 0:
            print("\nSUCCESS: All tests (Slow, Multi-pin, Config and Stress) passed with 0 errors!")
            print("The MCP23008 interrupt implementation is fully functional and robust.")
        else:
            print(f"\nWARNING: Stress test encountered {stress_errors} errors.")

finally:
    reset_state()
