# DD_ECAT_v3 — Fix Plan

Issues found in code review, ordered by severity. Items within each tier are independent unless noted.

---

## Tier 1 — Critical (fix first)

### 1. `VTIME=1` blocks every loop iteration up to 100 ms
**File:** `main.c` — `enable_raw_mode()`  
**Problem:** `VTIME=1` means `read()` in `check_ctrl_combinations()` waits up to 100 ms for a key. The main loop target is 10 ms. Loop timing is dominated by this single call.  
**Fix:** Change `raw.c_cc[VTIME] = 0` (fully non-blocking, VMIN=0 already set).

---

### 2. FFB effect type is never set in the HID parser
**File:** `hid_interface.c` — `parse_ffb_report()`  
**Problem:** `memset(effect, 0, ...)` zeroes the entire struct including `effect->type`. Neither report 2 nor report 3 parsing assigns `type`. All effects silently become `FFB_EFFECT_CONSTANT_FORCE (0)` — spring, damper, friction, etc. are unreachable.  
**Fix:** Map the raw `effect_type` byte from the HID report to the `ffb_effect_type_t` enum before returning.

---

### 3. Effects are consumed once and die — no persistent effect table
**File:** `main.c` main loop, `hid_interface.c` queue design  
**Problem:** DirectInput PID sends *set-and-forget* commands (Set Effect, then Effect Operation: Start). The queue pops one effect per 10 ms cycle; if the queue is empty, `effect_ptr = NULL` and torque = 0. A spring effect produces force for exactly one cycle then stops.  
**Fix:** Implement an effect slot table (indexed by effect block index, as per USB PID spec). Each slot stores the active effect and its state (running/stopped/duration elapsed). The calculator sums all running slots each cycle instead of consuming from a queue.

---

### 4. `pause_control` leaves motor energised at last torque
**File:** `main.c` main loop  
**Problem:** On pause, the loop `continue`s without writing a new torque. The EtherCAT thread keeps sending the last `target_torque_f` indefinitely. The wheel holds whatever force was active.  
**Fix:** Before `continue`, call `soem_interface_send_and_receive_pdo(0.0f)`.

```c
if (pause_control) {
    soem_interface_send_and_receive_pdo(0.0f);  // ADD THIS
    usleep(10000);
    continue;
}
```

---

### 5. Emergency stop is unreachable and self-clears in one cycle
**File:** `main.c`  
**Problem (a):** `EMERGENCY_STOP_THRESHOLD = 10000`, but both calculator versions clamp output to ±5000. The threshold can never be crossed.  
**Problem (b):** Even if triggered, `apply_safety_checks()` zeroes `desired_torque`. The end-of-loop check `fabs(desired_torque) < MAX_TORQUE_LIMIT * 0.5f` sees 0.0 and clears the flag on the same iteration.  
**Fix:** Lower the threshold to e.g. `4500` (within the clamped range). Latch `emergency_stop` — clear it only on explicit operator input (e.g. Ctrl+E), not automatically.

---

### 6. `case` labels with immediate declarations — compile error with `-std=c11`
**File:** `ffb_calculator.c` — `ffb_calculator_calculate_torque()`  
**Problem:** Cases `FFB_EFFECT_SPRING`, `DAMPER`, `INERTIA`, `FRICTION`, `PERIODIC` all declare local variables as the first statement after the label. C11 forbids this (label must precede a statement, not a declaration).  
**Fix:** Wrap each case body in `{ }` braces.

---

## Tier 2 — Correctness / design

### 7. `memset` on output PDO clears only 5 of 35 bytes
**File:** `soem_interface.c` — `soem_interface_init_enhanced()`  
**Problem:** `somanet_rx_pdo_enhanced_t` is 5 bytes, but the slave output SM is 35 bytes (`ec_slave[1].Obits/8`). Only the struct-covered prefix is zeroed; target position, velocity, torque offset, etc. are uninitialised `IOmap` memory.  
**Fix:** `memset(ec_slave[slave_idx].outputs, 0, ec_slave[slave_idx].Obits / 8)` before assigning the struct pointer.

---

### 8. Encoder bit-width labelled inconsistently throughout
**Files:** `soem_interface.c`, `main.c`, `synapticon_servo_tuning.h`  
**Problem:** Comments and log strings alternate between "14-bit" and "16-bit". `uint32_t encoder_increments = 65536.0f` assigns a `float` literal to a `uint32_t`.  
**Fix:** Settle on 16-bit (65536). Replace all "14-bit" strings with "16-bit". Use `(uint32_t)65536` in the SDO write.

---

### 9. Cycle-time mismatch: PDO thread vs interpolation-time SDO
**File:** `soem_interface.c`  
**Problem:** EtherCAT thread uses `cycle_time = 1000 µs` (1 ms / 1 kHz). Interpolation time period SDO 0x60C2 is written as **10 ms**. For CST mode the drive expects 0x60C2 to match the actual PDO cycle.  
**Fix:** Set interpolation time to 1 ms (`interpolation_time_period = 1`).

---

### 10. No encoder wrap/unwrap handling for multi-turn
**File:** `main.c` — `update_position_system()`  
**Problem:** If `position_actual_value` is single-turn absolute (0–65535), crossing zero produces a ±65536 jump → ±360° position step → violent spring torque spike.  
**Fix:** If the drive returns multi-turn int32, this is fine — confirm in drive documentation. If single-turn, unwrap: track the previous raw position and add ±65536 when the delta exceeds half-range.

---

### 11. HID descriptor is not a valid USB PID FFB descriptor
**File:** `create_ffb_gadget.sh`  
**Problem:** The descriptor is a flat list of vendor-ish output bytes. It lacks required PID structure: Set Effect Report (with effect-type usage array), Create New Effect, Block Load/Free, Pool, Effect Operation, Device Control, PID Device State reports. Windows DirectInput will not enumerate this as an FFB device.  
**Fix:** Replace the descriptor with a proven PID-compliant one (e.g. from OpenFFBoard or Hid-joystick-FFB-esp). This will also dictate the correct parse logic for issue 2/3.  
**Note:** This is a prerequisite for any game to send FFB commands.

---

## Tier 3 — Threading / performance

### 12. HID reception thread holds mutex across 5 ms `select()` + `read()`
**File:** `hid_interface.c` — `_usb_ffb_reception_thread()`  
**Problem:** `hidg_fd_mutex` is held for the entire select+read block. `hid_interface_send_gamepad_report()` (called from the RT main loop) must wait up to 5 ms to acquire it each cycle.  
**Fix:** Snapshot `fd = hidg_fd` under a brief lock, then release the mutex before `select()`/`read()`.

---

### 13. Gamepad report thread is an empty timing shell
**File:** `hid_interface.c` — `_gamepad_report_loop()`  
**Problem:** The thread does nothing except print stats every 1000 iterations. Reports are sent from the main loop via `hid_interface_send_gamepad_report()`. The thread wastes a core-2 pinning and a pthread.  
**Fix:** Remove the thread. Either keep reports in the main loop or move them into this thread (and remove the direct call from main).

---

### 14. CPU affinity doesn't match isolated core
**Files:** `hid_interface.c`, `soem_interface.c`, `README.md`  
**Problem:** `isolcpus=3` isolates core 3, but HID threads are pinned to cores 1 and 2. The EtherCAT thread (highest jitter requirement) and main loop have no affinity.  
**Fix:** Pin the EtherCAT thread to core 3 (the isolated core) with `SCHED_FIFO` priority 80+. Pin main to core 2. Leave HID threads on core 1 or unpinned.

---

### 15. CSV logging (`fprintf` + `fflush`) runs on the RT main thread
**File:** `main.c` — `log_ffb_data()`  
**Problem:** Six `fprintf` calls under a mutex per 10 ms cycle. Periodic `fflush` to SD card can block for tens of ms. `%ld` for `timestamp_ms` overflows on 32-bit hosts — use `int64_t` + `PRId64`.  
**Fix:** Write each log line with `snprintf` into a preallocated ring buffer (lock-free or per-slot); drain to file from a low-priority dedicated logger thread.

---

### 16. Unsynchronised shared state (data races)
**Files:** `soem_interface.c`, `hid_interface.c`, `main.c`  
**Affected variables:** `current_statusword`, `current_cia402_state` (read via getters without mutex); `usb_connected`, `consecutive_write_failures`, `last_reconnect_attempt`, `logging_enabled`, `log_counter`.  
**Fix:** Use C11 `_Atomic int` / `atomic_int` for flags and counters that are written from one thread and read from another without mutex coverage.

---

### 17. Distributed Clocks enabled but cyclic thread is free-running
**File:** `soem_interface.c`  
**Problem:** `ec_configdc()` is called but the thread sleeps with `clock_nanosleep(CLOCK_MONOTONIC, ...)` without aligning to the DC sync0 offset. Leads to drive sync warnings and higher jitter.  
**Fix:** After `ec_configdc()`, read the DC offset and align `next_wakeup` to it before entering the loop (standard SOEM DC example pattern).

---

## Tier 4 — Code quality / build

### 18. No header dependency tracking in Makefile
**File:** `Makefile`  
**Problem:** Editing a `.h` does not trigger recompilation of dependent `.o` files.  
**Fix:** Add `-MMD -MP` to `CFLAGS` and `-include $(OBJS:.o=.d)` at the bottom of the Makefile.

---

### 19. SOEM path hardcoded in Makefile
**File:** `Makefile`  
**Problem:** `/home/mwi/SOEM/install/...` only works on one specific machine.  
**Fix:** `SOEM_DIR ?= /home/mwi/SOEM/install` at the top; reference `$(SOEM_DIR)/include/soem` and `$(SOEM_DIR)/lib`.

---

### 20. `printf` in signal handlers is not async-signal-safe
**File:** `main.c` — `sigint_handler`, `sigusr1_handler`, `sigusr2_handler`  
**Problem:** `printf` is not async-signal-safe per POSIX — it can deadlock if the signal arrives while the main thread holds the FILE* lock.  
**Fix:** Remove `printf` from signal handlers. Set the flag only; let the main loop print status after it detects the flag change.

---

### 21. Dead declarations in `ffb_calculator.h`
**File:** `ffb_calculator.h`  
**Problem:** `ffb_calculator_update`, `ffb_calculator_process_effect`, `ffb_calculator_get_torque`, `ffb_calculator_set_gains` are declared but never defined or called. `ffb_calculator_init` is declared twice.  
**Fix:** Remove the unimplemented declarations and the duplicate `init` declaration.

---

### 22. `ffb_motor_effect_t` field overloading is fragile
**File:** `ffb_types.h`  
**Problem:** `direction` doubles as ramp end-magnitude; `start_delay` and `timestamp` carry packed periodic waveform parameters (waveform/frequency/phase/offset). Both `type`/`effect_type` and `duration_ms`/`duration` carry overlapping info.  
**Fix:** Add a `union` keyed by effect type, or add explicit named fields (`ramp_end`, `periodic_waveform`, etc.) and remove the packing hacks.

---

### 23. Build artifacts and log files committed to repo
Tracked files that should be in `.gitignore`:
- `*.sim.o` (`ffb_calculator_v2.sim.o`, `hid_interface.sim.o`, etc.)
- `ffb_sim` (binary)
- `soem_interface_v2` (binary)
- `ffb_log_*.csv`

---

## Tier 5. Documentation

### 24. Update readme.md
Update the readme.md file based on the changes made on to this plan.md focus on:
- How the user should setup everything when using this setup (software whise)
- Configure the raspberry pi for this repo usage. 
- How to deal with faults.

---

## Summary checklist

| # | File(s) | Issue | Done |
|---|---------|-------|------|
| 1 | `main.c` | `VTIME=1` blocks loop | [x] |
| 2 | `hid_interface.c` | `effect->type` never set | [x] |
| 3 | `hid_interface.c`, `main.c` | No persistent effect table | [x] |
| 4 | `main.c` | Pause leaves torque active | [x] |
| 5 | `main.c` | Emergency stop unreachable + auto-clears | [x] |
| 6 | `ffb_calculator.c` | Case-label declarations, C11 error | [x] |
| 7 | `soem_interface.c` | Partial PDO memset (5/35 bytes) | [x] |
| 8 | `soem_interface.c`, `main.c` | 14-bit vs 16-bit confusion | [x] |
| 9 | `soem_interface.c` | Interpolation time SDO wrong (10 ms → 1 ms) | [x] |
| 10 | `main.c` | No encoder wrap/unwrap | [x] |
| 11 | `create_ffb_gadget.sh` | HID descriptor not PID-compliant | [x] |
| 12 | `hid_interface.c` | Mutex held across 5 ms select | [x] |
| 13 | `hid_interface.c` | Empty gamepad report thread | [x] |
| 14 | `hid_interface.c`, `soem_interface.c` | CPU affinity vs isolated core mismatch | [x] |
| 15 | `main.c` | Blocking CSV log on RT thread | [x] |
| 16 | `soem_interface.c`, `hid_interface.c`, `main.c` | Data races on shared state | [x] |
| 17 | `soem_interface.c` | DC sync enabled but thread free-runs | [x] |
| 18 | `Makefile` | No header dependency tracking | [ ] |
| 19 | `Makefile` | SOEM path hardcoded | [ ] |
| 20 | `main.c` | `printf` in signal handlers | [ ] |
| 21 | `ffb_calculator.h` | Dead / duplicate declarations | [ ] |
| 22 | `ffb_types.h` | Field overloading in `ffb_motor_effect_t` | [ ] |
| 23 | `.gitignore` | Binaries and CSVs committed to repo | [ ] |
