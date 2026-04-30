# DD_ECAT_v3

## Schematic layout

- Based on: <https://github.com/Ultrawipf/OpenFFBoard/wiki>
- ![DD_ECAT_layout_v1](https://github.com/user-attachments/assets/eae2e943-441e-4428-8cb8-4dae76ef00f0)

## Concept idea

- I want to use a Raspberry Pi 4B, that can connect as a USB HID Joystick with FFB functionality. With the FBB commands from the game via something like: Windows.Gaming.Input.ForceFeedback.
- Using SOEM on my raspberry pi 4B to make it a EtherCAT master that sends commands to the Synapticon ACTLINK-S servo motor and sends the actual position (encoder value) back to SOEM.
- I have the PDO mapping available in the PDO_mapping.txt file.

## What i have now

- The script: create_ffb_gadget.sh makes the Pi to show up as a USB HID. But im not sure if the FFB inputs are correct.
- I created a basic hid_interface.c and .h that should get the right commmands from the games i am playing.
- This should send the commands to the ffb_calculator. These recalculated commands should be send to the SOEM interface.
- This SOEM interface start up the connection using EtherCAT with the servomotor and intergrated encoder en sends torque, position and velocity commands to the motor and reads the actual shaft position and sends this back via the SOEM interface to the FFB axis/effects calculator.
- All of this is regulated via the main.c script.

## To make it work

### Configure the pi

#### Phase 1: Raspberry Pi Setup

- Flash OS: Flash Raspberry Pi OS Lite (64-bit) to your SD card using the Raspberry Pi Imager.
- Initial Boot & Config:
Boot the Pi. Connect it to your network via its Ethernet port for now. SSH into it.
Run sudo raspi-config to set your locale, timezone, and enable SSH.

- Update your system:

sudo apt update
sudo apt full-upgrade -y

- Install Essential Tools:

sudo apt install -y build-essential git

- Isolate a CPU Core (Optional but Recommended): For better real-time performance, you can dedicate a CPU core to the main control loop. Edit the boot command line:

sudo nano /boot/cmdline.txt

Add isolcpus=3 to the end of the line. This reserves the 4th core (core #3) for our application. Reboot after saving.

#### Phase 2: EtherCAT Master Setup (SOEM)

- Clone SOEM:
cd ~
git clone <https://github.com/OpenEtherCATsociety/SOEM.git>

##### Build SOEM

- cd SOEM
- mkdir build
- cd build
- cmake ..
- make
- sudo make install

##### Configure Network Interface

The EtherCAT master needs raw socket access to the Ethernet port. We'll use eth0.
Find your Pi's MAC address: ip a (look for link/ether under eth0).
Connect the Synapticon drive directly to the Pi's Ethernet port. Power on the Synapticon drive (both logic and motor power).

##### Test Communication

SOEM comes with example tools. Let's use slaveinfo to see if the Pi can find the drive.
Navigate to the build directory where the test executables are: ~/SOEM/build/test/linux/slaveinfo/
Run the test. You MUST use sudo because it requires raw network access.

sudo ./slaveinfo eth0

If successful, you will see output like:
SOEM (Simple Open EtherCAT Master)
Slaveinfo
Starting slaveinfo
ec_init on eth0 succeeded.
1 slaves found and configured.
Slave 1
 MAN: 00000abc ID: 12345678 REV: 00000001
 State: PREOP
 ... (more info)
This step is critical. If it fails, do not proceed. Check your cabling, power, and network interface name.

#### Phase 3: Clone git project

- Clone DD_ECAT_v3:
- cd ~
- git clone <https://github.com/Mwi93/DD_ECAT_v3.git>
- cd DD_ECAT_v3
- make
- Check if the ffb_app is made correctly

##### to do

- Check via claude.ai (pro) all files if it works all together correctly.
- Debug SOEM interface, the LED ring stays red during operation. Indicating there is a mismatch in PDO mapping.
- Fix HID interface bug.

#### Phase 4: USB HID Gadget Setup

This phase makes the Pi appear as a joystick to your PC.

##### Enable libcomposite

- cd ~
- echo "dtoverlay=dwc2" | sudo tee -a /boot/config.txt
- echo "libcomposite" | sudo tee -a /etc/modules

##### Create the HID Gadget Script

We need a script that defines the joystick's capabilities (1 axis for steering, FFB support). Copy the file named create_ffb_gadget.sh to the correct location.

- cd DD_ECAT_v3
- sudo cp ./create_ffb_gadget.sh /usr/bin/create_ffb_gadget.sh
- sudo chmod +x /usr/bin/create_ffb_gadget.sh
- to test: sudo reboot

##### Automaticlly start the script

- use this command: sudo nano /etc/systemd/system/ffb-gadget.service
- Paste this:
{
[Unit]
Description=Create USB Gadget for FFB Wheel
After=network.target sys-kernel-config.mount
Requires=sys-kernel-config.mount

[Service]
ExecStart=/usr/bin/create_ffb_gadget.sh
Type=oneshot
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
}

- Than enable and start it:
{sudo chmod +x /usr/bin/create_ffb_gadget.sh
sudo systemctl daemon-reexec
sudo systemctl enable ffb-gadget.service
sudo reboot
}

#### Phase 5: Compile all files

- cd DD_ECAT_v3
- make -f Makefile_v2
- If correctly installed:
- sudo ./ffb_app eth1

#### Phase 6: testing and improving
- sudo cpufreq-set -g performance
- sudo nano /boot/config.txt
- add or replace: gpu_mem=128
- add #include "synapticon_servo_tuning.h" to excisting files
- Add these in main.c:
- // Add after soem_interface_init_enhanced():
- configure_synapticon_for_steering_wheel(1);
- print_synapticon_status(1);

---

## Simulation mode — no Raspberry Pi or Synapticon motor needed

If you don't have a Raspberry Pi or a Synapticon servo motor available, you can run the full software stack on any Linux PC (x86/x64) using the built-in simulation mode.

The simulation replaces the real EtherCAT/SOEM layer (`soem_interface.c`) with a software motor model (`soem_interface_sim.c`). The rest of the stack — FFB calculator, HID interface, safety checks, logging — runs completely unchanged.

**What the simulation models:**
- 1-DOF rotational system: `I·α = torque - b·ω`
- Moment of inertia, viscous damping, and rated torque are configurable constants at the top of `soem_interface_sim.c`
- Soft virtual end-stops at ±540° (matching the real steering range)
- Encoder position reported in 65 536 counts/revolution (same as the real 16-bit Synapticon encoder)
- Always reports `CIA402_STATE_OPERATION_ENABLED` so the control loop starts immediately

---

### Prerequisites (Linux PC)

You need a standard Linux desktop or VM with:

```bash
sudo apt install build-essential gcc libpthread-stubs0-dev
# librt and libm are part of glibc and are always available
```

No SOEM, no EtherCAT drivers, no special kernel needed.

---

### Step 1 — Build the simulation binary

From the project directory:

```bash
make -f Makefile_v2 sim
```

This produces `./ffb_sim`. It uses `-DSIM_MODE` which replaces the SOEM include with a stub, so no EtherCAT library is required.

---

### Step 2 — Set up the USB HID gadget (optional)

The HID interface writes to `/dev/hidg0`. On a PC that file does not exist unless you configure a USB OTG gadget.

**Option A — Skip HID, test FFB calculations only**

If you only want to verify the FFB physics and logging, comment out the HID start call in `main.c`:

```c
// hid_interface_start();   // comment out for headless simulation
```

Rebuild with `make -f Makefile_v2 sim`. The simulation will run, log to a CSV file, and print position/velocity/torque to the terminal. No USB connection needed.

**Option B — Full HID test with a second Linux machine**

1. On the PC that runs `ffb_sim`, install the `libcomposite` gadget using the same `create_ffb_gadget.sh` script from the repo (requires a board with USB OTG support, such as a Pi, or a VM with USB passthrough).
2. Connect via USB to a Windows PC. The device will appear as a joystick in Windows Game Controllers and accept FFB commands from any DirectInput game.

---

### Step 3 — Run the simulation

```bash
sudo ./ffb_sim sim_eth0
```

The interface name argument is ignored in simulation mode — you can pass any string.

`sudo` is needed for:
- `SCHED_FIFO` real-time scheduling (`mlockall` in `setup_real_time_scheduling`)
- Writing to `/dev/hidg0` if HID is active

Expected startup output:

```
=== Raspberry Pi FFB Steering Wheel Application ===
Synapticon 16-bit Absolute Encoder Version with FFB Logging
...
SIM: soem_interface_init_enhanced() — simulation mode, no real hardware
SIM: Motor model: I=0.050 kg*m^2, b=2.0 Nms/rad, rated=10.0 Nm
SIM: Encoder: 65536 counts/rev, steering limit ±540°
SIM: Physics thread started at 1000 Hz
SIM: Initialisation complete — simulation running
...
Starting optimized servo control loop at 1000 Hz...
Status: Deg=0.0° (0.000 rev), Vel=0.0°/s, Torque=0.0, EtherCAT=OK, HID=...
```

---

### Step 4 — Tune the simulation parameters

Open `soem_interface_sim.c` and adjust the constants at the top to match your actual hardware:

| Constant | Default | Description |
|---|---|---|
| `SIM_INERTIA` | `0.05` | Moment of inertia in kg·m². Increase for a heavier wheel. |
| `SIM_DAMPING` | `2.0` | Viscous damping in Nms/rad. Increase for more resistance. |
| `SIM_RATED_TORQUE_NM` | `10.0` | Motor rated torque in Nm (1000 per-mille = this value). |
| `SIM_MAX_ANGLE_DEG` | `540.0` | Steering lock-to-lock half-angle in degrees. |
| `SIM_STOP_STIFFNESS` | `50.0` | Spring stiffness of virtual end-stops in Nm/rad. |

After changing, rebuild with `make -f Makefile_v2 sim`.

---

### Step 5 — Read the log file

Every run creates a timestamped CSV log file in the current directory (e.g. `ffb_log_20260430_143000.csv`). Open it in Excel, LibreOffice Calc, or Python/pandas to inspect:

- Wheel position and velocity over time
- Every FFB effect the game sent (type, magnitude, coefficients)
- Calculated torque output
- Emergency stop events

Use `Ctrl+L` while running to toggle logging on/off, and `Ctrl+R` to recenter the wheel position.

---

### Keyboard controls (both simulation and real hardware)

| Key | Action |
|---|---|
| `Ctrl+C` | Graceful shutdown |
| `Ctrl+R` | Recenter wheel (set current position as new zero) |
| `Ctrl+L` | Toggle CSV logging on/off |

