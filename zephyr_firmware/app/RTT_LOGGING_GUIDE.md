# RTT Logging Setup Guide

## Prerequisites

Install OpenOCD and GDB for ARM:

```bash
sudo pacman -S openocd arm-none-eabi-gdb
```

## How to Use RTT Logging

### Step 1: Build and Flash Your Firmware

Build as usual:
```bash
cd /home/yeet/Projects/brage_telemetry/zephyr_firmware/app
# Your normal build command here
# west build -b your_board
# or cmake/ninja commands
```

Flash to your board using your normal method.

### Step 2: Start OpenOCD with RTT

In one terminal, start OpenOCD with RTT server:

```bash
openocd -f interface/stlink.cfg -f target/stm32u5x.cfg \
  -c "init" \
  -c "rtt setup 0x20000000 0x40000 \"SEGGER RTT\"" \
  -c "rtt server start 9090 0" \
  -c "rtt start"
```

You should see output like:
```
Info : Listening on port 9090 for rtt connections
```

### Step 3: Connect to RTT Logs

In another terminal:

```bash
telnet localhost 9090
```

Or use netcat:
```bash
nc localhost 9090
```

You'll now see your log output in real-time!

## Troubleshooting

### OpenOCD Can't Find RTT Control Block

If OpenOCD shows "RTT: No control block found", the address range might be wrong. Try:

```bash
# For STM32U5 with 256KB RAM starting at 0x20000000
openocd -f interface/stlink.cfg -f target/stm32u5x.cfg \
  -c "init" \
  -c "rtt setup 0x20000000 0x40000 \"SEGGER RTT\"" \
  -c "rtt server start 9090 0" \
  -c "rtt start" \
  -c "rtt polling_interval 100"
```

### No Output Appearing

1. Make sure your firmware is running (check with debugger)
2. Verify RTT is enabled in prj.conf
3. Try increasing buffer size: `CONFIG_SEGGER_RTT_BUFFER_SIZE_UP=2048`
4. Check log level is set correctly: `CONFIG_SX128X_LOG_LEVEL_DBG=y`

### Alternative: Use pyOCD

If OpenOCD doesn't work well, try pyOCD:

```bash
# Install
pip install pyocd

# Start with RTT
pyocd gdbserver --target stm32u575cgtx --rtt
```

Then connect to RTT on port 19021 by default.

## Log Output Format

You should see output like:

```
[00:00:00.123,456] <inf> sx128x_hal: Initializing SX128x HAL
[00:00:00.123,567] <dbg> sx128x_hal: SPI device ready
[00:00:00.123,678] <dbg> sx128x_hal: CS GPIO configured
[00:00:00.123,789] <inf> sx128x_hal: SX128x HAL initialized successfully
```

## Switching Back to UART

If you want to use physical UART instead, edit `prj.conf`:

```kconfig
# Comment out RTT
# CONFIG_USE_SEGGER_RTT=y
# CONFIG_LOG_BACKEND_RTT=y

# Enable UART
CONFIG_LOG_BACKEND_UART=y
CONFIG_UART_CONSOLE=y
```

Connect USB-to-UART to PA2 (TX), PA3 (RX), GND and use:
```bash
minicom -D /dev/ttyUSB0 -b 115200
```

## Tips

- **RTT is faster** than UART (can handle more log traffic)
- **No extra wires** needed - uses existing debug connection
- **Works while debugging** - you can use GDB and see logs simultaneously
- **Immediate mode** means logs appear instantly (no buffering)
