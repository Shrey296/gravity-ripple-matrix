# gravity-ripple-matrix

A physics simulation of ripple waves on an 8×8 LED matrix, controlled by an accelerometer. Tilt or shake the board and waves form, bounce off the walls, and slowly die out on their own. Leave it alone, and the surface goes completely dark with no looping animations and idle patterns.

---

## Demo

---

## Working

Most LED matrix "wave" projects are just looping sine animations - they look like waves but don't respond to anything. This one runs the actual **2D wave equation** on the ESP32, the same physics that describes water surfaces and drum membranes:

```
velocity += c² × laplacian(height) − damping × velocity + force
height   += velocity
```

The `force` term comes from the MPU6050 accelerometer, but not from raw tilt - it uses **jerk** (change in acceleration between frames). This means a board sitting still on a tilted surface reads as zero force, and the surface stays calm. Only actual movement disturbs it.

The ESP32 splits the work across both cores using FreeRTOS: physics runs on Core 1 at ~60 fps, and LED rendering runs on Core 0. They share the wave grid behind a mutex so neither blocks the other.

LED brightness uses **Binary Code Modulation** across 8 sub-frames: wave crests glow at full brightness, and everything else fades proportionally. The overall matrix intensity also tracks total wave energy in real time, so as the last ripples die out, the whole display dims with them.

---

## Parts

| Part | What I used |
|------|------------|
| Microcontroller | ESP32-WROOM-32 dev board |
| Accelerometer | MPU-6050 |
| Display | 8×8 LED matrix with MAX7219 driver |
| Everything else | Jumper wires |

---

## Wiring

### MPU6050 → ESP32

| MPU6050 | ESP32 |
|---------|-------|
| VCC | 3V3 |
| GND | GND |
| SDA | D21 |
| SCL | D22 |
| XDA, XCL, AD0, INT | leave unconnected |

### MAX7219 module → ESP32

| MAX7219 | ESP32 |
|---------|-------|
| VCC | VIN (5V) |
| GND | GND |
| DIN | D23 |
| CLK | D18 |
| CS | D5 |

Both GND pins on the ESP32 are identical - put one module on each if you want

---

## Libraries

All three available in Arduino IDE's Library Manager:

- **MD_MAX72XX** by majicDesigns - MAX7219 driver
- **MPU6050_light** by rfetick - lightweight MPU6050 interface
- **Wire** - built-in

---

## Flashing

1. Add the ESP32 board package to Arduino IDE if you haven't already
2. Select **Tools → Board → ESP32 Dev Module**
3. Install the libraries above
4. Wire everything up
5. Upload the sketch
6. Open Serial Monitor at **115200 baud**
7. Hold the board flat and still for a second while it calibrates
8. Give it a nudge

If the upload fails, hold **BOOT** on the ESP32 while clicking Upload and release once it says `Connecting...`

---

## Tuning

The behaviour is almost entirely controlled by four constants at the top of the sketch. These are the ones worth playing with first:

| Constant | Default | What it does |
|----------|---------|--------------|
| `C2` | `0.18` | Wave speed - lower feels like thick water, higher feels snappy |
| `DAMPING` | `0.005` | How fast waves die - raise this for a quick-settling puddle feel |
| `FORCE` | `0.10` | How hard the accelerometer kicks the surface - raise if gentle tilts do nothing |
| `lc.control(INTENSITY, 10)` | `10` | Base brightness, 0-15 |
