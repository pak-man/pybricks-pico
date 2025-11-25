# WiFi Configuration Guide

## WiFi Modes

### THREADSAFE_BACKGROUND (Default - Recommended)
**Best for:** General use, easy setup
```bash
make WIFI_MODE=THREADSAFE_BACKGROUND
```
- Automatic WiFi handling in background
- No manual polling required
- Slight latency increase (~5-10ms)
- Works well with motor control

### POLL
**Best for:** Real-time applications, minimal latency
```bash
make WIFI_MODE=POLL
```
- Manual `cyw43_arch_poll()` required
- Lowest latency
- More control over timing
- Need to call poll() every 1-10ms

### SYS (FreeRTOS)
**Best for:** Complex applications with multiple tasks
```bash
make WIFI_MODE=SYS
```
- FreeRTOS-based
- True multitasking
- More memory overhead
- Most flexible

## Quick Start

1. **Edit WiFi credentials** in `main.c`:
```c
#define WIFI_SSID "YourNetwork"
#define WIFI_PASSWORD "YourPassword"
```

2. **Copy files** to `bricks/picow/`:
   - CMakeLists.txt
   - lwipopts.h  
   - main.c
   - Makefile
   - web/ directory

3. **Build**:
```bash
cd bricks/picow
make picow  # Uses default THREADSAFE_BACKGROUND mode
```

4. **Flash** the .uf2 file

5. **Access** web interface at displayed IP address

## File Structure

```
bricks/picow/
├── CMakeLists.txt        # Build config with WiFi mode selection
├── lwipopts.h            # lwIP configuration
├── main.c                # Main with WiFi init
├── Makefile              # Build automation
├── web/
│   ├── index.html        # Web UI
│   └── app.js            # JavaScript
├── system_test.c         # BLE commands
└── btstack_port.c        # Bluetooth
```

## Mode Comparison

| Feature | THREADSAFE_BG | POLL | SYS |
|---------|---------------|------|-----|
| Ease of use | ⭐⭐⭐ | ⭐⭐ | ⭐ |
| Latency | ~10ms | <1ms | ~5ms |
| CPU usage | Low | Minimal | Medium |
| Memory | 8KB | 8KB | 12KB |
| Motor control | ✅ | ✅ | ✅ |
| BLE concurrent | ✅ | ✅ | ✅ |
| Multitask | ❌ | ❌ | ✅ |

## Switching Modes

Clean rebuild required when changing modes:
```bash
make clean
make picow WIFI_MODE=POLL
```

## Troubleshooting

**WiFi won't connect:**
- Check SSID/password
- Verify 2.4GHz network (5GHz unsupported)
- Check WPA2-PSK auth

**BLE stops working:**
- In POLL mode: ensure `cyw43_arch_poll()` called frequently
- Check main loop not blocking

**Web page slow:**
- THREADSAFE_BG has ~10ms latency
- Use POLL mode for faster response

**Build errors:**
- Verify `PICO_SDK_PATH` set
- Check lwIP libraries available in SDK
- Clean and rebuild

## Advanced

### Static IP
Add after `cyw43_arch_init()`:
```c
ip4_addr_t ip, mask, gw;
IP4_ADDR(&ip, 192,168,1,100);
IP4_ADDR(&mask, 255,255,255,0);
IP4_ADDR(&gw, 192,168,1,1);
netif_set_addr(netif_list, &ip, &mask, &gw);
```

### Custom HTTP handlers
Edit CGI/SSI handlers in main.c to add endpoints.

### Memory tuning
Adjust `lwipopts.h` for your application:
- `MEM_SIZE`: Total heap
- `MEMP_NUM_TCP_PCB`: Max connections
- `TCP_WND`: TCP window size
