# Open serial monitor (minicom, screen, or your preferred tool)
screen /dev/ttyACM0 115200
# or
minicom -D /dev/ttyACM0 -b 115200
```

You should see startup messages. Look for:
```
Pybricks Pico W starting...
Firmware version: 1.0.0
```

### Test via BLE (preferred method):

1. **Connect via nRF Connect:**
   - Scan for "Pybricks Hub" or "Pybricks Pico W"
   - Connect
   - Find Nordic UART Service (NUS)
   - Enable TX notifications

2. **Send test command:**
```
   DSP VERIFY
```

3. **Expected output on serial console:**
```
   === RP2350 DSP Verification ===
   ✓ ARM_MATH_CM33 defined
   ✓ ARM_MATH_DSP defined
   ✓ __ARM_FEATURE_DSP=1

   Testing DSP hardware instructions...
   ✓ DSP SIMD instructions working (SADD16 test passed)

   Benchmarking MAC operations...
     Scalar: XXXX µs
     DSP:    YYYY µs
     Speedup: Z.ZZx
   ✓ DSP extensions providing significant acceleration
```

4. **Send benchmark command:**
```
   DSP BENCH
```

   **Expected BLE response:**
```
   DSP: Running benchmark...
   DSP Benchmark Results:
     Scalar: XXXX cycles
     DSP:    YYYY cycles
     Speedup: Z.ZZx
