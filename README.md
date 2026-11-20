# levelup联赛半自动及自动赛 — Team 86000A

Driver-control, semi-autonomous and autonomous code for a **VEX IQ (2nd gen)** robot, team **86000A**, for the VEX IQ *Level Up* season.

Code comments are in Chinese, with English translations on the line below each one (`// EN: ...`).

## Project layout

| Path | What it holds |
|---|---|
| [src/main.cpp](src/main.cpp) | Entry point, driver controls, background tasks, the qualification and skills routines, and the brain screen display |
| [src/self-pro.cpp](src/self-pro.cpp) | Motion library: heading PID, drive-at-heading, drive-for-distance/time, turn-to-angle |
| [src/config.cpp](src/config.cpp) | Motor and sensor port assignments |
| [include/self-pro.h](include/self-pro.h) | Motion library API (`run_dir_speed*`, `turnto`, `move`, `PIDController`) |
| [include/config.h](include/config.h) | `extern` declarations for the devices |
| `vex/`, `makefile` | VEXcode build system (normally left alone) |

## Building and downloading

1. Install VS Code with the **VEX Robotics** extension.
2. Open this folder. The extension reads [.vscode/vex_project_settings.json](.vscode/vex_project_settings.json): platform **IQ2**, language C++, project name `simple-86000A`.
3. Connect the brain or controller over USB, then use **Build** / **Download** in the VEX extension. The program goes to **slot 2**.

Build output goes to `build/`, which git ignores.

## Hardware configuration

Defined in [src/config.cpp](src/config.cpp):

| Device | Name in code | Port | Reversed |
|---|---|---|---|
| Arm | `arm` | 1 | no |
| Touch LED | `TouchLED` | 2 | — |
| Front distance sensor | `frontdis` | 4 | — |
| Back distance sensor | `backdis` | 5 | — |
| Intake / conveyor | `intake` | 6 | no |
| Left drive motor | `leftM` | 7 | yes |
| Launcher motor (left) | `injL` | 8 | no |
| Right drive motor | `rightM` | 10 | no |
| Launcher motor (right) | `injR` | 12 | yes |
| Launcher group | `inj` | `injL` + `injR` | — |
| Inertial sensor | `BrainInertial` | built into the brain | — |

Ports 3, 9 and 11 are free.

Drivetrain: 2.5 in wheels, 1:2 motor-to-wheel gearing. Distances in the motion functions are in inches (`IN2M_DEG = 22.5` motor degrees per inch, tuned by hand).

## Starting up

When the program starts, the brain:

1. Sets the heading to 0. Inertial calibration is commented out in `main()`, so keep the robot still and square at startup.
2. Shows a **mode menu**: pick **Starter** or **Automation** with the brain's arrow buttons and confirm with Check. With no input for 3 seconds it starts in Starter.
3. Homes the mechanisms. The arm and launcher run until they stall (or for at most 2 seconds each, so a disconnected or miswired motor can't freeze startup), then the arm encoder is zeroed.
4. Starts three background tasks: `autorun` (waits for an auto routine to be triggered), `chas` (joystick driving) and `L4thred` (arm buttons and screen display).

The brain screen shows the team number, the heading, both distance sensor readings and the current `run_shootdis` (in Starter Mode, the last row shows the mode instead).

## Controls

There are two modes, chosen at startup.

### Starter Mode

Simple driver controls with no automation: no autonomous routines, no distance-sensor driving and no steering assist.

| Input | Action |
|---|---|
| Joysticks | Arcade drive (Axis A forward/back, Axis C turn) |
| L Up (press and release) | Conveyor on, forward |
| L Down (hold) | Conveyor in reverse; stops on release |
| R Up (tap) | Arm up to the top level, then back down |
| R Up (hold 0.5 s) | Arm to 270° and hold there, the safe height for firing |
| R Down | Arm all the way down |
| F Up | Fire, then re-arm. Only works when the arm is above 200°; otherwise the brain beeps |
| F Down | Cock the catapult |
| E Up or E Down (hold 0.5 s) | Stop all motors and end the program |

The rest of this section describes **Automation Mode**, which is the original competition code.

### Driving (manual mode)

| Input | Action |
|---|---|
| Axis A | Forward / back. When the robot faces about −90° (−105° to −75°) and the stick is pushed hard, a ±13 offset is added to keep it driving straight |
| Axis C | Turn |
| L Up (press and release) | Intake on, forward |
| L Down (hold) | Intake in reverse; stops on release |
| R Up | Next arm level (`L4++`): raise to place, then lower. From level 3 on, the arm goes to 270° instead |
| R Down | Set level to 2 and lower the arm |
| F Up | Shoot and drive forward. Only works when `L4 > 1` and arm > 200° |
| F Down | Shoot and drive back to collect. The path depends on how many times F Down has been pressed (`fdcount` = 1, 2, 3+). Same conditions as F Up |
| E Up (hold > 2 s, arm > 200°) | Retract the launcher and **stop the program** |

F Up and F Down both use the back distance sensor to pick the shot. If the robot is already closer than `run_shootdis + 10` mm, it backs up against the bar and shoots. Otherwise it reverses to `run_shootdis` and shoots while moving.

### Autonomous

1. **Press the Touch LED** (it turns green). This resets the heading to 0, homes the arm, and resets `L4` and `fdcount`.
2. **Keep holding it for more than 1 s, then let go.** The LED turns blue and auto is armed.
3. Start a routine:
   - **F Up** runs `autoqualification()`, the league (teamwork) routine.
   - **Touch LED** or **E Down** runs `autoskill()`, the skills routine. It runs the qualification routine first, then keeps scoring.
4. **To abort**, push Axis A forward past 50, or either way past 90. The LED turns red and control goes back to the driver.

## Tuning guide

| What | Where |
|---|---|
| Run-and-shoot distance (mm, back sensor) | `run_shootdis` at the top of [src/main.cpp](src/main.cpp) |
| Straight-driving heading PID | `set_heading_pid(p, i, d)` in `main()`. Routines also change it mid-run |
| Turn PID (`turnto`) | `tkp`, `tki`, `tkd`, `tkf` in `main()` |
| Inches → motor degrees | `IN2M_DEG` in [src/self-pro.cpp](src/self-pro.cpp) |
| Semi-auto paths after game pieces 11–13 | The `fdcount == 1 / 2 / else` branches in the F Down handler |
| Routines | `autoqualification()` and `autoskill()` in [src/main.cpp](src/main.cpp) |

### Motion library cheat sheet

```cpp
run_dir_speed(heading, speed);                  // one step of PID-corrected driving; call it inside your own loop
run_dir_speed_dis(heading, speed, inches, stop); // drive a distance, slowing over the last 15%
run_dir_speed_time(heading, speed, seconds, stop);
turnto(heading);                                // turn PID to an absolute heading
move(left, right);                              // raw drive, percent
```

Headings are absolute inertial `rotation()` degrees. Negative speeds drive backwards. With `stop = false` the robot keeps moving at the last speed, which lets you chain moves smoothly.

## Notes

- A joystick-triggered semi-auto shot (Axis A < −80 with the back sensor < 25 mm) is in the code but commented out, inside the F Up / F Down `if` conditions. Remove the comment markers to turn it on.
- The routines often reset the heading with `BrainInertial.setRotation(-83 or -87, degrees)` after squaring up against the bar. If you change those, re-check every heading that comes after.
- The launcher fires asynchronously: the routines set `autoinjstate = true` and the main loop runs `autoinj()`.
