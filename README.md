# 03-stm32-robotic-arm

RT-Thread application layer for an STM32-based competition robotic arm. The repository contains only the
author's `User/Task` code: the RT-Thread threads that sequence a two-pass pick-and-place mission (material
area → roughing area → finishing area), drive four serial-bus (Feetech SCS) servos forming the arm and claw,
control a four-wheel Mecanum chassis with encoder-wheel odometry plus PID/soft-goal tracking, and talk to a
vision module, an LCD/button UI, a PS2 remote control and ultrasonic ranging boards. It is the application
layer of a teaching-kit firmware: the BSP, the Function layer, the servo protocol library, the CAN slave-board
firmware, the STM32 HAL and RT-Thread are **not** here, so it does not build standalone (see *How to run / build*).

## What it does / scope

- **Mission and arm sequencing** – `PathWrite_task` walks a fixed waypoint list, calls vision-based alignment
  and runs arm pose sequences to place material in the roughing area, then the finishing area, twice. Every
  arm motion is `WritePosEx(servo_id, position, time, speed)` on the SCS serial bus (IDs 1–4 = rotate, arm
  front/back, arm up/down, claw); there is no inverse kinematics, poses are recorded constants.
- **Chassis control** – `rccu_task` runs a 100 Hz loop with mode state machine
  `CHASSIS_RELAX / STOP / NORMAL / COORD / TRACKING`, three position PIDs (X, Y, yaw) and three
  `Location_Tracker` soft-goal generators; wheel RPMs go to four CAN DC-motor slave boards.
- **Perception and HMI** – serial protocol handling for a vision host (colour, QR/colour-sequence, circle
  centre) and a 4-channel HC-SR04 ranging module; 8-page LCD UI with 5 buttons and a buzzer, a TM1652
  3-digit battery gauge, and a PS2 gamepad thread mapping stick values to chassis velocities.
- **Out of scope** – slave-board motor/servo control loops, the servo protocol itself, the vision firmware,
  RT-Thread and the HAL.

## Repository layout

`firmware_arm_control` (variant A) and `firmware_arm_integrated` (variant B) share the same 18-file structure;
files marked *same* are byte-identical, so per-file differences are named below.

```
firmware_arm_control/User/Task/      # variant A - active PathWrite_task path is only the Run_1..Run_5 demo
firmware_arm_integrated/User/Task/   # variant B - active PathWrite_task path is the full two-pass mission
  inc/data.h                 # app/module data, flash struct, chassis + servo macros (1100 vs 1300 mm/s limits)
  inc/Servo.h                # arm-action prototypes (variant A also declares Run_1..Run_8)
  inc/thread_comm.h          # same - feature switches (EncodingWheel, Disable_gyro_WhenArmMOVE), shared state
  inc/thread_comm_test.h     # same - empty header skeleton
  inc/thread_gui.h           # same - lcd_page + BUSPosition_Ctrl_Scan()
  inc/thread_led.h           # same - empty header skeleton
  inc/thread_rccu.h          # same - chassis / servo-control API declarations
  inc/thread_remotectrl.h    # same - empty header skeleton (reuses the thread_gui.h include guard)
  inc/thread_user.h          # same - Task_User_create(), ResetCoordinateXYJ()
  src/data.c                 # factory flash block, CRC16 validation/restore, board init export
  src/Servo.c                # arm postures and pick/place functions (poses, claw range, Get_B/Run_* differ)
  src/thread_comm.c          # comms thread, UART callbacks, encoder-wheel frame parser (differs)
  src/thread_comm_test.c     # same - standalone UART/CAN passthrough test (symbol clash with thread_comm.c)
  src/thread_gui.c           # same - LCD pages, key events, slave-board read-out
  src/thread_led.c           # same - TM1652 battery gauge thread
  src/thread_remotectrl.c    # same - PS2 pad to sRemoteCtrl_Info velocities
  src/thread_rccu.c          # chassis control loop, PIDs + trackers, mode machine (indentation differs)
  src/thread_user.c          # mission sequencer thread (active branch, QR sweep, axis mapping differ)
```

There are no build files (`*.uvprojx`, `Makefile`, `CMakeLists.txt`), no linker scripts and no `.git` data.

## How it works

1. **Boot.** `INIT_BOARD_EXPORT(AppData_Init)` (`src/data.c`) validates the factory flash block
   (`FH1=0x5A`, `FH2=0xA5`, `Servo_RotationClawInit=1420`, `Servo_Claw_S=1750`, `Servo_Claw_J=1300`,
   `Bit16CRC=0x4203`) and rewrites it when blank or when its CRC16/Modbus check fails. `INIT_APP_EXPORT`
   registers `Task_COMM_create`, `Task_RCCU_create`, `Task_GUI_create`, `Task_LED_create`,
   `Task_RemoteCtrl_create`.
2. **Comms bring-up and loop.** `Task_COMM_create` initialises the CAN slave boards (4× DC motor, 1× 3-channel
   steering, 4× tracking), `CANCommunication_Init()`, four UARTs with callbacks (`muart2` vision, `muart3` QR
   module, `muart4` encoder wheel, `muart6` second QR port), the vision module, four HC-SR04 channels and the
   EXTI/timer handlers, then starts `comm` (priority 1). `comm_task` loops over `CANCommunication_Scan()`,
   `RecognitionModule_Scan1Ms()` for both module instances and `Application_DataFlash_Modification()`, and
   every 20 iterations pushes `AngleValue[]`/`AngleTime[]` to a 6-channel steering board.
3. **Chassis bring-up.** `rccu_init()` zeroes `rccu_struct`, selects `CHASSIS_RELAX`, points
   `qGyro_YawAngle_New` at the encoder-wheel yaw `zangle` (the optional `HWT101_gyro` path is compiled out),
   calls `Chassis_Init(FOUR_DRIVE_McNamara, …)`, and initialises the yaw/X/Y PIDs, three `Location_Tracker`
   generators and `LineTracker_Init`; `Task_RCCU_create` starts `rccu` (priority 2).
4. **Chassis loop.** `rccu_task` waits 1 s, latches the yaw zero, starts in `CHASSIS_RELAX`, calls
   `Task_User_create()`, then every ~10 ms reads the yaw, runs the current `mode_run` branch and calls
   `rccu_chassisctrl()`: chassis kinematics → wheel RPMs → four DC boards over CAN → read back of speed and
   odometry into `Chassis_Calc_ForwardTransform`. Entering `CHASSIS_COORD` re-latches soft/goal targets;
   wheel output is gated on `lcd_page == 1`.
5. **Vision data in.** `XferExternalUart2Rx_Handler` → `RecognitionModule_ReceivingProcess(&PCModule_t)`. On
   `RM_succeed` the payload is copied into `Recognition_Buffer[20]`; `RM_Colorstart` maps byte 0 to
   `CurrentMaterialColor` (`'1'/'2'/'3'` = `RED/GREEN/BLUE`); `RM_Location`/`RM_Circle` split `(x,y)` with
   `atol` into `cartesian_loc.x/.y`; `bool_recognitionflag` is set to 1.
6. **Encoder-wheel data in.** `XferExternalUart4Rx_Handler` feeds each byte to `Data_Analyse`, a 5-state
   parser for `0x0D 0x0A` + 24 bytes + `0x0A 0x0D` frames read as six floats: `ActVal[0] → zangle`,
   `ActVal[3] → pos_x`, `ActVal[4] → pos_y`, then `Wheel_isReady = 1`. The same handler extracts
   `"start<digits>"` → `StartTemperature` and `"check"` → `CalibratResults`.
7. **Ready gate.** The GUI thread turns `Wheel_isReady == 1` into a buzzer cue, `"Click K5 to Start"` and
   `Wheel_isReady = 2`; `Wait_Start()` blocks until that, and `Debug_Await()` until `KEY_5` is pressed on
   `lcd_page == 1`.
8. **Mission (variant B).** `PathWrite_task` → `Wait_Start()`, then `ChassisCoord_Set(...)` +
   `ChassisCoord_WaitStop()` waypoints: start → material area → `IdentifyColor_action()` +
   `IdentifyColor_Get(0)` (three pick/place cycles in `ColorSequence[0]` order) → roughing area →
   `IdentifyCircle_Get(100,150)` → `RoughingArea_PutGet(0)` → `SetCoordinateXY(0,0)` → finishing area →
   `IdentifyCircle_Get(100,150)` → `FashioningArea_Put(0)` → material area (index 1) → start area.
9. **Arm motion and visual alignment.** Each pick/place is a named posture (`IdentifyColor_action`,
   `IdentifyCircle_action`, `Rest_action_S2`, …) plus colour/bowl functions (`Get_A`, `Get_1..3`,
   `Get_Red/Green/Blue`, `Put_1..3`, `Put_*_Stacking`) built from `WritePosEx` calls and settling delays;
   `Disable_gyro_WhenArmMOVE` switches the chassis to `CHASSIS_RELAX` around arm motion so reaction torque
   does not corrupt localisation. `IdentifyCircle_Get` loops on `cartesian_loc` every ~10 ms with a P + D
   correction (`1.5 * err + 1.5 * (err - err_prev)`), clamps it to ±50, and exits at ≤ 2 mm error or after
   1500 iterations, calling `Rest_action_S2()`.
10. **Manual control and HMI.** `remotectrl_task` polls the PS2 pad every ~49 ms and converts sticks with
    `PS2_RESOLUTION = 128` into `sRemoteCtrl_Info.sPosition.vx/vy` (`RC_CHASSIS_MAX_SPEED_X/Y`) and `.vyaw`
    (`RC_GIMBAL_MOVE_RATIO_YAW = 0.12`), consumed by `rccu_chassisctrl_normalmode`. `gui_task` refreshes the
    LCD, debounces five keys via `mybtn_*` and reads motor speeds, tracking sensors and servo positions;
    `led_task` (or the idle hook under `RT_USING_IDLE_HOOK`) drives the TM1652 gauge (`Battery_Max = 12.6 V`,
    `Battery_Min = 11.1 V`).

## Key functions and modules

| File | Function / Class | Purpose |
| --- | --- | --- |
| `User/Task/src/thread_user.c` | `PathWrite_task`, `Task_User_create` | Mission thread (waypoints, alignment, pick-and-place passes) and its thread creation |
| | `Debug_Await`, `Wait_Start` | Wait for `KEY_5` on LCD page 1 / for `Wheel_isReady == 2` |
| | `IdentifyColor_Get`, `IdentifyCircle_Get`, `Color_Get`, `Circle_Get` | Vision-triggered pick sequence and P+D circle-centre alignment |
| | `RoughingArea_PutGet`, `FashioningArea_Put`, `*_R` variants | Ring placement: fixed red/green/blue order, or the recognised `CircleSequence` |
| | `SetCoordinateXY`, `ResetCoordinateXYJ` | Re-sync / zero the encoder-wheel frame over `muart4` (`ACTX/ACTY/ACTJ/ACT0`) |
| `User/Task/src/Servo.c` | `Rest_action`, `Rest_action_S2`, `QR_action`, `IdentifyColor_action`, `IdentifyCircle_action` | Named arm postures |
| | `Get_A`, `Get_B`, `Get_1..3`, `Get_Red/Green/Blue`, `Put_1..3`, `Put_Red/Green/Blue`, `Put_*_Stacking` | Pick/place primitives (material area, bowls, rings, stacking) |
| | `Run_1..Run_8` (variant A only) | Demo pose sequence; `Run_6..Run_8` are empty stubs |
| `User/Task/src/thread_rccu.c` | `rccu_task`, `rccu_init` | 100 Hz chassis loop and one-time initialisation |
| | `rccu_chassisctrl_normalmode`, `_coordmode`, `_trackingmode` | PID + soft-goal modes, and the line-tracking mode |
| | `ChassisCoord_Set`, `ChassisCoord_WaitStop`, `ChassisSpeed_Set`, `ChassisRELAX_set`, `ChassisModle_Set`, `Read_Position_*` | Public chassis API and odometry read-back |
| `User/Task/src/thread_comm.c` | `comm_task`, `Task_COMM_create`, `Data_Analyse` | 1 ms comms/init thread, encoder-wheel frame state machine (5 states, 24-byte payload) |
| | `XferExternalUart2/3/4/6Rx_Handler`, `EXTI_Callback`, `TimeBreakExecution_Handler`, `Timer2UpdateBreakExecution_Handler` | Receive, HC-SR04 echo and timer callbacks |
| `User/Task/src/thread_gui.c` | `gui_task`, `Task_GUI_create`, `Display_Window_*`, `Display_123`, `DCMotor_Ctrl_Scan`, `TrackingSensor_Ctrl_Scan`, `BUSPosition_Ctrl_Scan` | LCD refresh, page/line navigation, key handling, slave-board read-out |
| `User/Task/src/data.c` | `AppData_Init`, `Application_DataFlash_Modification` | Flash factory block: CRC16/Modbus validation and default restore |
| `User/Task/inc/data.h` | `FlashData_Typedef`, `AppData_Typedef`, `GyroData_Typedef`, `_s_RemoteCtrl_Info`, `BrickData_TypeDef`, `RM_cartesian` | Shared data structures and chassis/servo macros |
| `User/Task/src/thread_led.c`, `thread_remotectrl.c` | `led_task` / `vApplicationIdleHook`, `Task_LED_create`, `PS2RC_ReadData`, `remotectrl_task` | TM1652 battery gauge; PS2 pad decoding into `sRemoteCtrl_Info` |
| `User/Task/src/thread_comm_test.c` | `comm_task`, `Task_COMM_create`, `XferExternalUart*Rx_Handler` | Standalone loopback/bridge test; same symbols as `thread_comm.c` |

## Dependencies

All items below are included by the code but **absent from this repository**; the code records no version numbers for any of them.

- **Vendor BSP layer** – `bsp.h` (`Bsp_UartMixed_Init/TxTrigger`, `Bsp_Cancom_FilterInit`, `EXTI_SetHandler`, `Timer_SetHandler`, `Timer2Update_SetHandler`, `CANCommunication_Init/Scan`, `CANSend`, `CAN_OutQueue`, `My_mDelay`, `HAL_xmsDelay`, `Echo1..4_read`, `Trig1..4_level`, `ECHO1..4_Pin`, `KEY_1..KEY_5`, `BatVoltage_Read`, `buzzerSound`), `usart.h`, `separate_button.h`, `separate_led.h`, `RGB.h`, `lcd.h`.
- **Vendor Function layer** – `mytype.h`, `CheckCalc.h` (`CRC16Modbus_Compute`, `checksum`, `stmflash_write`), `RecognitionModule.h` (`RM_*` states, `RecognitionModule_Start/Stop/Scan1Ms/ReceivingProcess`, `ColorRecognitionModule_Start`, `CircleRecognitionModule_Start`), `HC_SR04.h` (`sHCSR04Data`, `HCSR04_Inti`, `HCSR04Work_Scan`, `HCSR04_TimeOverflow`, `HCSR04_DistanceCalc`, `HCSR04_PulseWidthRead`), `chassis_function.h` (`Chassis_Init`, `FOUR_DRIVE_McNamara`), `chassis_LineTracker.h` (`LineTracker_Init`, `LineTracker_Scan`), `Location_Tracker.h`, `pid.h` (`PID_struct_init`, `POSITION_PID`), plus the CAN slave-board drivers `SLAVE_DCMotorMiniwatt`, `SLAVE_SteeringEngine_3CH`, `SLAVE_Tracking` (guarded by `__SLAVE_DCMotorMiniwatt_H__`, `__SLAVE_SteeringEngine_3CH_H__`, `__SLAVE_Tracking_H__`, `__SLAVE_SteeringEngine_6CH_H__`).
- **SCSLib (Feetech serial-bus servo protocol)** – `SCServo.h`, source of the only arm primitive used, `WritePosEx(servo_id, position, time, speed)`.
- **STM32 HAL / CMSIS** – `stm32f4xx.h`, `usart.h`, `HAL_Delay`, `__IO`, `TIM2`.
- **RT-Thread** – `rtthread.h`, `rthw.h`: `rt_thread_create`, `rt_thread_startup`, `rt_kprintf`, `RT_NULL`, `rt_enter_critical`/`rt_exit_critical`, `rt_hw_interrupt_disable/enable`, `INIT_BOARD_EXPORT`, `INIT_APP_EXPORT`, `RT_USING_IDLE_HOOK`, `rt_thread_idle_sethook`.
- **C library / toolchain** – `string.h` (`strstr`, `strlen`, `memset`), `stdlib.h` (`atol`), `stdio.h`, `stdint.h`, `stdbool.h`.

## How to run / build

**This repository cannot be built or run standalone.** It holds only the `User/Task` application layer of a
teaching-kit firmware: there is no project file, no Makefile/CMakeLists, no linker script, no startup code and
none of the libraries listed above. To build it you need the vendor teaching-kit firmware project for the
STM32F4 board (RT-Thread based, with its BSP, Function and `SLAVE_*` layers and its `bsp.h` pin/macro
configuration), the Feetech SCSLib library, the STM32 HAL/CMSIS package, an RT-Thread source tree or prebuilt
library, and the Keil/IAR/MDK toolchain used by that kit. Then:

1. Copy the chosen variant's `User/Task/` directory into the vendor project, replacing the kit's own `User/Task`.
2. Add every `src/*.c` — **exactly one** of `thread_comm.c` **or** `thread_comm_test.c` (they define the same
   `thread_comm`, `XferExternalUart*Rx_Handler` and `Task_COMM_create` symbols).
3. Add `User/Task/inc/` plus all vendor include paths, and define the `__SLAVE_*` module switches exactly as
   the vendor project does.
4. Build, flash and run: the buzzer/`"Click K5 to Start"` prompt appears once the encoder wheel reports ready;
   press `KEY_5` while the LCD shows page 1 to start the mission.

No exact command line can be given, because no toolchain, board target or project file is present here.

## Provenance and attribution

Only the author's `User/Task` application layer is included in this repository. The teaching-kit-provided BSP
and Function layers, the SCSLib serial-servo protocol library (Feetech), the CAN slave-board firmware, the
STM32 HAL and RT-Thread itself are third-party code written by others and are **not** included here; they
remain under their own licences and must be obtained separately. The two firmware variants are the author's own
`User/Task` sources and differ only in the files marked in *Repository layout*.

## Limitations and known issues

Nothing here compiles or runs as-is (see *How to run / build*), and no build, board or clock configuration is
recorded.

- **Header-guard and linkage defects.** `inc/thread_remotectrl.h` reuses the include guard `__THREAD_GUI_H_`,
  so whichever of it and `inc/thread_gui.h` is included first suppresses the other's contents; and
  `ResetCoordinateXYJ` is declared non-static in `inc/thread_user.h` but defined `static` in
  `src/thread_user.c` — an ISO C constraint violation a strict compiler rejects.
- **`thread_comm.c` and `thread_comm_test.c` cannot coexist** in one image (duplicate symbols), yet both are
  shipped side by side.
- **`Application_DataFlash_Modification()` always returns 0**, although its comment documents "1: success,
  0: failure"; the local `succeed` is never set to 1.
- **Missing timeouts.** `IdentifyColor_Get` has no global timeout (`while(1)` exits only after three
  successful matches), and variant B's `Color_Get` contains two consecutive
  `while (Color_wait == CurrentMaterialColor)` spins that can block forever if the material colour never
  changes.
- **Serial buffer assumptions.** `XferExternalUart4Rx_Handler` calls `strstr(pdata, "start")`, which needs a
  NUL-terminated buffer it does not produce (the test handlers write up to `pdata[len+6]`), and
  `XferExternalUart2Rx_Handler` copies a fixed `Recognition_RX_LEN` (20) bytes regardless of the real length.
- **The vision axis mapping is orientation-dependent.** Comments state the chassis axes are swapped/negated
  depending on which way the vision module faces, and the variants use different expressions
  (`ChassisSpeed_Set(xout,-yout)` in variant A vs `(-yout,-xout)` in variant B), so they cannot both be
  correct for the same hardware orientation.
- **Variant A's active path is not the mission.** `firmware_arm_control`'s `PathWrite_task` compiles only
  `Run_1(); … Run_5();`; the full two-pass mission, and with it most of that file's helpers
  (`IdentifyColor_Get`, `Color_Get`, `Identify_QRCode`, `RoughingArea_*`, `FashioningArea_*`,
  `SetCoordinateXY`), sit in the disabled `#else` branch. Its `Run_6..Run_8` are empty stubs, and its
  `Servo.h` declares `Get_B`, which its `Servo.c` never defines.
- **Unused or missing definitions.** `ServoSetPluseAndTime()` is declared in `inc/thread_rccu.h` but defined
  nowhere here; the servo channel macros in `inc/data.h` (`SERVO_CLAW*`, `SERVO_ARM1/2`, `SERVO_YAW`,
  `DEVICE_ROBOTARM_ID`) are referenced by no included source (servo IDs are passed as literals 1–4).
- **Nothing here supports a performance claim.** No measurements, results, photos, wiring notes or logs are
  included; pose numbers are undocumented and differ between variants (claw range 800/1100/1370 in variant A vs
  2300/2000/1600 in variant B), with no units given for any pose value.
- **The vision protocol is only inferred** from the parsers (`'1'/'2'/'3'` = red/green/blue, `(<x>,<y>)` =
  coordinates); the module's own specification is not part of the repository.
