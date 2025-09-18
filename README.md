# BMP390 Kernel Driver - Pressure sensor BMP390

This is a Linux kernel driver for the **Bosch BMP390** pressure and temperature sensor, integrated with the Industrial I/O (IIO) subsystem. The driver supports both I2C and SPI interfaces, offering full compatibility with the Bosch BMP390 datasheet and feature parity with the user-space LibDriver implementation. It provides robust functionality for temperature and pressure measurement, calibration, oversampling, output data rate (ODR), IIR filtering, FIFO, interrupt handling, and power management, making it suitable for production-grade embedded systems.

## Introduction

The BMP390 driver is designed for Linux kernel environments, targeting platforms such as Raspberry Pi (BCM2835/BCM2711). It leverages the IIO subsystem to expose sensor data through sysfs and device files, supports hardware interrupts for real-time data acquisition, and includes advanced power management for efficiency. The driver is modular, with separate components for core logic, I2C/SPI interfaces, and IIO integration, ensuring maintainability, scalability, and reliability.

### Key Enhancements
- **Power Management**: Runtime PM and system suspend/resume for efficient power usage.
- **Thread Safety**: Mutexes and wait queues for concurrent access and real-time data handling.
- **Interrupt Handling**: Support for data-ready and FIFO watermark/full interrupts with workqueue processing.
- **Device Tree Support**: Flexible configuration via Device Tree for operation mode, FIFO, and ODR.
- **Sysfs Attributes**: Extended attributes for oversampling, sampling frequency, and FIFO watermark.
- **Kernel Thread**: Dedicated kernel thread for periodic data collection in periodic mode.
- **IOCTL Interface**: Enhanced IOCTL commands for mode control, ODR, FIFO flush, and calibration data retrieval.
- **Documentation**: Comprehensive kernel-doc for all functions and structures.

## Features
- **Interfaces**: Supports I2C (addresses 0x76/0x77) and SPI (chip select 0, up to 1 MHz).
- **Measurements**: Provides raw (`in_temp_raw`, `in_pressure_raw`) and processed (`in_temp_input`, `in_pressure_input`) data in °C and Pa.
- **Calibration**: Loads calibration parameters from NVM and applies Bosch’s polynomial compensation algorithms.
- **Configurability**:
  - Oversampling ratios: x1, x2, x4, x8, x16, x32 for temperature and pressure.
  - Output Data Rate (ODR): 0.78 Hz, 1.5 Hz, 3 Hz, 6 Hz, 12.5 Hz, 25 Hz, 50 Hz, 100 Hz.
  - IIR filter coefficients: 0, 1, 3, 7, 15, 31, 63, 127.
  - FIFO: Configurable watermark (1–255) and data source (filtered/unfiltered).
  - Interrupts: Data-ready, FIFO watermark/full, push-pull/open-drain, active high/low.
  - SPI: 3-wire or 4-wire mode.
  - I2C: Configurable watchdog period.
- **Sysfs Integration**: Attributes for oversampling (`in_temp_oversampling_ratio`, `in_pressure_oversampling_ratio`), sampling frequency (`in_sampling_frequency`), and FIFO watermark.
- **Interrupt Handling**: Hardware interrupts via GPIO (e.g., GPIO17/18) with threaded IRQ support.
- **Device Tree**: Configurable via Device Tree overlay for operation modes (`default`, `periodic`, `async`), FIFO watermark, ODR, and shared memory size.
- **Power Management**: Runtime PM for dynamic power control and system suspend/resume for deep sleep.
- **Kernel Thread**: Dedicated thread for periodic data collection in `periodic` mode.
- **IOCTL**: Commands for setting operation mode, ODR, flushing FIFO, and retrieving calibration data.

## Driver Structure
The driver is organized into modular components for clarity and maintainability:
1. **`bmp390.h`**: Defines registers, enums (oversampling, ODR, filter coefficients, operation modes), data structures (`bmp390_data`, `bmp390_shared_data`, `bmp390_calib_param`), and function prototypes with kernel-doc.
2. **`bmp390_core.c`**: Core logic for register access (with retries), calibration, compensation, oversampling, ODR, IIR filter, FIFO, interrupt configuration, and kernel thread management.
3. **`bmp390_i2c.c`**: I2C-specific probe, remove, initialization, interrupt setup, and power management (runtime PM, suspend/resume).
4. **`bmp390_spi.c`**: SPI-specific probe, remove, initialization, interrupt setup, and power management (runtime PM, suspend/resume).
5. **`bmp390_iio.c`**: IIO subsystem integration for raw/processed data, triggered buffer, sysfs attributes, and interrupt-driven data handling.
6. **`bmp390.c`**: Main driver entry point, handling module initialization, I2C/SPI driver registration, misc device setup, and IOCTL interface.
7. **`bmp390_test.c`**: User-space test program for validating driver functionality (raw/processed data, IOCTL, sysfs, and interrupt testing).
8. **`bmp390_driver.dts`**: Device Tree overlay for configuring I2C/SPI interfaces, interrupts, and sensor parameters on Raspberry Pi.
9. **`Makefile`**: Build system for compiling kernel modules, device tree overlay, and test program, with support for module signing and documentation generation.

## Performance Enhancements
- **Optimized Register Access**: Implemented retry mechanism (up to 3 attempts with 10ms delay) for I2C/SPI communication to handle transient errors, reducing latency and improving throughput.
- **Efficient Data Processing**: Streamlined compensation algorithms in `bmp390_compensate_temp_press` using 64-bit arithmetic for high precision with minimal CPU overhead.
- **Kernel Thread Optimization**: Dedicated kernel thread (`bmp390_kthread_fn`) for periodic mode uses wait queues and completions to minimize polling overhead and ensure low-latency data collection.
- **FIFO Efficiency**: Configurable FIFO watermark and data source reduce unnecessary data transfers, with interrupt-driven buffer pushes to IIO for high throughput.
- **Runtime PM**: Dynamic power state transitions in `read_raw`, `write_raw`, `irq_handler`, and other functions reduce CPU wake-ups and optimize power usage during idle periods.
- **Buffer Management**: IIO triggered buffer with timestamp alignment ensures efficient data streaming for real-time applications.

## Security Enhancements
- **Thread Safety**: Mutexes protect all shared resources (registers, shared memory) to prevent race conditions in multi-threaded environments.
- **Input Validation**: Strict validation of Device Tree properties (`operation-mode`, `fifo-watermark`, `output-data-rate`) and IOCTL arguments to prevent invalid configurations.
- **Memory Safety**: Use of `kzalloc` for shared memory allocation and proper cleanup with `kfree` to avoid memory leaks.
- **IOCTL Security**: IOCTL commands include magic number checks (`BMP390_IOC_MAGIC`) to prevent unauthorized access, with `copy_from_user`/`copy_to_user` for safe user-space data transfer.
- **Resource Cleanup**: Comprehensive resource deallocation in probe/remove paths to prevent dangling pointers or resource leaks during driver unload.

## Scalability Enhancements
- **Modular Design**: Separation of core, I2C, SPI, and IIO components allows easy adaptation for other Bosch sensors (e.g., BMP580) or additional interfaces.
- **Device Tree Flexibility**: Supports multiple instances (e.g., I2C addresses 0x76/0x77, SPI chip select 0) via Device Tree, enabling multi-sensor configurations.
- **Configurable Parameters**: Extensive sysfs and IOCTL interfaces allow runtime configuration of oversampling, ODR, and FIFO settings, supporting diverse use cases.
- **Shared Memory**: Configurable shared memory size (`shared-memory-size`) in Device Tree supports scalability for varying data throughput needs.
- **Platform Compatibility**: Device Tree overlay compatible with multiple platforms (BCM2835, BCM2711), with potential for broader hardware support.

## Reliability Enhancements
- **Error Handling**: Comprehensive error checking with detailed logging (`dev_err`, `dev_dbg`) for all I2C/SPI operations, IOCTL calls, and interrupt handling.
- **Retry Mechanism**: Register read/write retries with exponential backoff (10ms delay) to handle communication failures gracefully.
- **Interrupt Robustness**: Threaded IRQ with workqueue processing ensures reliable data handling under high interrupt loads.
- **Power Management**: Runtime PM and suspend/resume ensure stable operation during power state transitions, with regulator enable/disable for VDD.
- **Resource Cleanup**: Proper cleanup of IIO buffers, kernel threads, shared memory, and regulators during driver removal or error conditions.
- **Testing**: Enhanced user-space test program (`bmp390_test.c`) validates all driver features, including raw/processed data, IOCTL, sysfs, and interrupts.

## Installation
### Prerequisites
- **Kernel Source**: Linux kernel build directory (`/lib/modules/$(uname -r)/build`).
- **Device Tree Compiler**: `dtc` for generating Device Tree overlays.
- **Cross-Compiler**: Optional for cross-platform builds (e.g., Raspberry Pi, use `CROSS_COMPILE` in Makefile).
- **Hardware Setup**:
  - I2C (pins SDA/SCL) or SPI (pins MOSI/MISO/SCK/CS) enabled on the target platform.
  - GPIO pin (e.g., GPIO17/18) connected for interrupts, as specified in Device Tree.
  - VDD regulator (3.3V) connected to the BMP390.
- **Dependencies**: IIO subsystem enabled in the kernel (`CONFIG_IIO`).

### Step-by-Step Installation
## OPTION 1:
1. **Clone the Repository** (if hosted):
   ```bash
   git clone <repository_url>
   cd bmp390-driver
   ```

2. **Set Up Environment** (optional for cross-compilation):
   ```bash
   export KERNEL_DIR=/path/to/kernel/source
   export CROSS_COMPILE=arm-linux-gnueabihf-
   export SIGN_KEY=/path/to/signing_key
   ```

3. **Build the Driver and Test Program**:
   ```bash
   make
   ```
   This compiles:
   - Kernel modules: `bmp390.o`, `bmp390_core.o`, `bmp390_iio.o`, `bmp390_i2c.o`, `bmp390_spi.o`.
   - Device Tree overlay: `bmp390_driver.dtbo`.
   - Test program: `bmp390_test`.

4. **Install Modules and Overlay**:
   ```bash
   sudo make install
   ```
   - Copies modules to `/usr/lib/modules`.
   - Installs `bmp390_driver.dtbo` to `/boot/overlays`.
   - Updates module dependencies with `depmod -a`.

5. **Configure Device Tree** (for Raspberry Pi):
   Edit `/boot/config.txt` to include:
   ```bash
   dtoverlay=bmp390_driver
   ```
   Reboot to apply:
   ```bash
   sudo reboot
   ```

6. **Load Kernel Modules**:
   ```bash
   sudo modprobe bmp390
   ```

7. **Verify Installation**:
   Check for IIO device:
   ```bash
   ls /sys/bus/iio/devices/
   ```
   Expected output: `iio:device0` (or similar).
   Verify module loading:
   ```bash
   lsmod | grep bmp390
   ```

8. **Generate Documentation** (optional):
   ```bash
   make doc
   ```
   Creates `bmp390_doc.rst` with kernel-doc for all functions and structures.

### Troubleshooting Installation
- **Module Not Loaded**: Ensure `depmod -a` was run and check `dmesg` for errors:
  ```bash
  dmesg | grep bmp390
  ```
- **Device Tree Issues**: Verify `bmp390_driver.dtbo` is in `/boot/overlays` and `dtoverlay=bmp390_driver` is in `/boot/config.txt`.
- **Interrupt Issues**: Confirm GPIO pin connections and Device Tree interrupt settings.

## OPTION 2:
### Step 1: Enable I2C or SPI
- Enable the desired interface:
  - Run `sudo raspi-config`, navigate to *Interfacing Options*, and enable *I2C* and/or *SPI*.
  - Alternatively, ensure `dtparam=i2c_arm=on` or `dtparam=spi=on` in `/boot/config.txt`.
- Reboot the Pi:
  ```
  sudo reboot
  ```

### Step 2: Prepare Device Tree Overlay
The `bmp390_driver.dts` file is a device tree overlay that enables the BMP390 sensor. Compile and apply it as follows:

- Navigate to the repository directory containing the source files.
- Compile the device tree source to a device tree blob overlay:
  ```
  make dtb
  ```
  This generates `bmp390_driver.dtbo`.

- Copy the overlay to the Raspberry Pi's overlay directory:
  ```
  sudo cp bmp390_driver.dtbo /boot/overlays/
  ```

- Edit `/boot/config.txt` to apply the overlay:
  ```
  sudo nano /boot/config.txt
  ```
  Add the following line at the end:
  ```
  dtoverlay=bmp390_driver
  ```
  - For I2C, ensure `dtparam=i2c_arm=on` is present.
  - For SPI, ensure `dtparam=spi=on` is present.

- Reboot to apply the overlay:
  ```
  sudo reboot
  ```

**Advanced (Optional)**: If the overlay does not work or you need to modify the base device tree:
- Navigate to `/boot`.
- Decompile the base DTB to DTS for your Pi model (e.g., for Raspberry Pi 4, use `bcm2711-rpi-4-b.dtb`):
  ```
  dtc -I dtb -O dts -o temp.dts bcm2711-rpi-4-b.dtb
  ```
- Edit `temp.dts` with a text editor:
  - Find the `i2c1` or `spi0` node.
  - Add the BMP390 node from `bmp390_driver.dts` (e.g., `bmp390@76` for I2C or `bmp390@0` for SPI).
- Recompile the DTS back to DTB:
  ```
  dtc -I dts -O dtb -o bcm2711-rpi-4-b.dtb temp.dts
  ```
- Reboot:
  ```
  sudo reboot
  ```

### Step 3: Build the Driver and Test Program
- Install kernel headers if not already present:
  ```
  sudo apt update
  sudo apt install raspberrypi-kernel-headers
  ```
- Ensure `KERNEL_DIR` in the Makefile points to your kernel source (default: `/lib/modules/$(uname -r)/build`).
- Build all artifacts (modules, overlay, and test program):
  ```
  make all
  ```
  This generates:
  - Kernel modules: `bmp390.ko`, `bmp390_core.ko`, `bmp390_iio.ko`, `bmp390_i2c.ko`, `bmp390_spi.ko`.
  - Device tree overlay: `bmp390_driver.dtbo`.
  - Test program: `bmp390_test`.

- To build specific components:
  - Kernel modules only: `make modules`
  - Device tree overlay only: `make dtb`
  - Test program only: `make bmp390_test`

- To clean build artifacts (keep source and generated artifacts):
  ```
  make clean
  ```
- To clean all generated files (including artifacts):
  ```
  make cleanall
  ```
- To generate documentation (requires kernel-doc):
  ```
  make doc
  ```
  This creates `bmp390_doc.rst`.

### Step 4: Install the Driver
- Install the kernel modules:
  ```
  sudo make install
  ```
  This installs modules to `/usr/lib/modules`, copies the DTBO to `/boot/overlays`, and runs `depmod -a`.

- Alternatively, manually load the modules:
  ```
  sudo insmod bmp390_core.ko
  sudo insmod bmp390_iio.ko
  sudo insmod bmp390_i2c.ko  # For I2C interface
  sudo insmod bmp390_spi.ko  # For SPI interface
  sudo insmod bmp390.ko      # Main module
  ```

- Verify installation:
  ```
  dmesg | grep BMP390
  lsmod | grep bmp390
  ```
  Look for messages like "BMP390 driver registered" or module names in `lsmod`.

- The driver exposes:
  - Misc device: `/dev/bmp390`
  - IIO device: `/sys/bus/iio/devices/iio:device0/`

### Step 5: Run the Test Program
The `bmp390_test.c` program demonstrates reading temperature/pressure, setting modes, and using ioctls.

- Run the test program (default: 3 readings, 1000ms interval):
  ```
  ./bmp390_test
  ```
- Specify custom number of readings and interval (e.g., 5 readings, 500ms interval):
  ```
  ./bmp390_test 5 500
  ```
- Example output includes:
  - Temperature and pressure (raw and processed).
  - Calibration parameters.
  - Tests for periodic and async modes.
  - FIFO flush confirmation.


## Usage
### 1. Reading Sensor Data
- **Via Sysfs**:
- Access sensor data and configure settings via sysfs:
  Read processed temperature and pressure:
  ```bash
  cat /sys/bus/iio/devices/iio:device0/in_temp_input
  cat /sys/bus/iio/devices/iio:device0/in_pressure_input
  ```
  Example output:
  ```
  Temperature: 23.450000 C
  Pressure: 101325.000000 Pa
  ```
- Read raw values:
  Read raw data:
  ```bash
  cat /sys/bus/iio/devices/iio:device0/in_temp_raw
  cat /sys/bus/iio/devices/iio:device0/in_pressure_raw
  ```
- Set pressure oversampling (e.g., x16):
  ```
  echo 16 > /sys/bus/iio/devices/iio:device0/in_pressure_oversampling_ratio
  ```
- Set sampling frequency (e.g., 25 Hz):
  ```
  echo 25 > /sys/bus/iio/devices/iio:device0/sampling_frequency
  ```
- Set FIFO watermark:
  ```
  echo 1 > /sys/bus/iio/devices/iio:device0/buffer/watermark
  ```
  
- **Via Device File**:
  Read raw buffer data (requires buffer enabled):
  ```bash
  cat /dev/iio:device0
  ```

### 2. Configuring Sensor Settings
- **Set Oversampling**:
  ```bash
  echo 16 > /sys/bus/iio/devices/iio:device0/in_pressure_oversampling_ratio
  echo 8 > /sys/bus/iio/devices/iio:device0/in_temp_oversampling_ratio
  ```
- **Set Sampling Frequency**:
  ```bash
  echo 25 > /sys/bus/iio/devices/iio:device0/in_sampling_frequency
  ```
- **Set FIFO Watermark**:
  ```bash
  echo 1 > /sys/bus/iio/devices/iio:device0/buffer/watermark
  ```
- **Available Options**:
  - Oversampling: `1`, `2`, `4`, `8`, `16`, `32`
  - Sampling frequency: `0.78`, `1.5`, `3`, `6`, `12.5`, `25`, `50`, `100` Hz
  - Filter coefficients: `0`, `1`, `3`, `7`, `15`, `31`, `63`, `127`
  - FIFO watermark: `1` to `255`

### 3. Enabling FIFO and Interrupts
- Enable IIO buffer:
  ```bash
  echo 1 > /sys/bus/iio/devices/iio:device0/buffer/enable
  ```
- Read FIFO data:
  ```bash
  cat /dev/iio:device0
  ```
- Ensure GPIO17/18 is connected (per Device Tree). The driver handles data-ready and FIFO watermark/full interrupts, pushing data to IIO buffers with timestamps.

### 4. Using the Test Program
Compile and run:
```bash
gcc bmp390_test.c -o bmp390_test -Wall -Wextra
./bmp390_test [number_of_readings] [interval_ms]
```
Example:
```bash
./bmp390_test 10 1000
```
Output:
```
BMP390 Kernel Driver Test
Set pressure oversampling to 16
Set sampling frequency to 25 Hz
Set FIFO watermark to 1
Temperature: 23.450000 C
Pressure: 101325.000000 Pa
Async: Temp=23450 C, Press=101325 Pa, Time=1234567890
FIFO flushed
Raw Temperature: 23450
Raw Pressure: 101325
Test completed
```

### 5. IOCTL Commands
The test program uses IOCTL to:
- Set operation mode (`BMP390_IOC_SET_MODE`): `default` (0), `periodic` (1), `async` (2).
- Set ODR (`BMP390_IOC_SET_ODR`): 0.78 Hz to 100 Hz.
- Flush FIFO (`BMP390_IOC_FLUSH_FIFO`).
- Retrieve calibration data (`BMP390_IOC_GET_CALIB`).
- Fetch shared memory data (`BMP390_IOC_GET_SHARED_DATA`).

Example in `bmp390_test.c`:
```c
int mode = 1; /* BMP390_OP_MODE_PERIODIC */
ioctl(fd, BMP390_IOC_SET_MODE, &mode);
```

### 6. Device Tree Configuration
Customize `bmp390_driver.dts`:
- `operation-mode`: `default`, `periodic`, `async`.
- `fifo-watermark`: 1 to 255.
- `output-data-rate`: 0.78 Hz to 100 Hz (mapped to 0–7).
- `shared-memory-size`: e.g., 0x1000.
- `interrupts`: e.g., `<17 1>` for GPIO17, rising edge.

Compile and apply:
```bash
make dtb
sudo cp bmp390_driver.dtbo /boot/overlays/
```

### 7. Debugging
- Check kernel logs:
  ```bash
  dmesg | grep bmp390
  ```
- Verify sysfs attributes:
  ```bash
  ls /sys/bus/iio/devices/iio:device0/
  ```
- Monitor interrupts:
  ```bash
  cat /proc/interrupts | grep bmp390
  ```
## Uninstallation
- Remove the kernel modules:
  ```
  sudo rmmod bmp390 bmp390_spi bmp390_i2c bmp390_iio bmp390_core
  ```
- Remove the device tree overlay:
  ```
  sudo dtoverlay -r bmp390_driver
  ```
- Reboot to ensure clean removal:
  ```
  sudo reboot
  ```
- Clean all generated files:
  ```
  make cleanall
  ```

## Troubleshooting
- **No device detected**:
  - Check `dmesg` for errors (e.g., "Invalid chip ID").
  - Verify hardware connections:
    - I2C: Run `i2cdetect -y 1` to check for addresses 0x76/0x77.
    - SPI: Ensure proper wiring and chip select.
  - Confirm power supply (3.3V via `vdd-supply` in DTS).
- **Build errors**:
  - Ensure kernel headers match the running kernel: `uname -r`.
  - Install headers: `sudo apt install raspberrypi-kernel-headers`.
- **Module loading fails**:
  - Check module dependencies: `depmod -a`.
  - If signed modules are required, set `SIGN_KEY` in Makefile.
- **Test program errors**:
  - Ensure `/dev/bmp390` exists and modules are loaded.
  - Check permissions: `sudo chmod +rw /dev/bmp390`.


## Advanced Features
- **Periodic Mode**: Kernel thread (`bmp390_kthread_fn`) collects data at configured ODR, storing in shared memory for user-space access.
- **Async Mode**: SIGIO signals for asynchronous data notifications via interrupts.
- **FIFO Parsing**: Interrupt-driven FIFO data with watermark support, pushed to IIO buffers with timestamps.
- **Power Management**: Runtime PM reduces power during idle; suspend/resume manages VDD regulator.
- **Error Handling**: Register access retries, detailed logging, and resource cleanup.
- **Thread Safety**: Mutexes and wait queues ensure reliable concurrent access.

## Notes
- **Upstream Status**: As of September 2025, the BMP390 may be supported in the mainline kernel via the `bmp280` driver with patches. This standalone driver is ideal for out-of-tree use but could be merged into `bmp280` for upstream submission.
- **Limitations**:
  - FIFO frame parsing is basic; raw data provided without full frame extraction.
  - Some sysfs attributes (e.g., FIFO watermark/source) are placeholders.
- **Future Improvements**:
  - Add debugfs for diagnostics (register dumps, calibration data).
  - Implement full FIFO frame parsing.
  - Enhance runtime PM with finer-grained power states.
  - Support additional Bosch sensors (e.g., BMP580).

## Acknowledgments
- Inspired by Linux kernel `bmp280` driver and community patches (2023–2025).
- Thanks to the Linux kernel community for IIO subsystem documentation and driver examples.
