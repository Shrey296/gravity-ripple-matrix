#include <MD_MAX72xx.h>
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#include <MPU6050_light.h>
#include <Wire.h>

// ─── Pin definitions ───────────────────────────────────
#define DIN_PIN   23
#define CLK_PIN   18
#define CS_PIN     5
#define SDA_PIN   21
#define SCL_PIN   22

// ─── Objects ───────────────────────────────────────────
MD_MAX72XX lc(HARDWARE_TYPE, DIN_PIN, CLK_PIN, CS_PIN, 1);
MPU6050 mpu(Wire);

// ─── Wave state (shared across cores) ──────────────────
const int N = 8;
volatile float h[N][N]   = {};
volatile float vel[N][N] = {};
portMUX_TYPE waveMux  = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE accelMux = portMUX_INITIALIZER_UNLOCKED;
volatile float shared_ax = 0, shared_ay = 0;
volatile float shared_energy = 0.0f;
portMUX_TYPE energyMux = portMUX_INITIALIZER_UNLOCKED;

// ─── Tuning ─────────────────────────────────────────────
//  C2       wave propagation speed² - keep under 0.5
//  DAMPING  energy drain per step - higher = dies faster
//  FORCE    how hard tilt/shake kicks the wave
const float C2      = 0.05f;
const float DAMPING = 0.005f;
const float FORCE   = 0.10f;

// ─── BCM brightness ─────────────────────────────────────
//  4 sub-frames per physics frame → 4 perceived brightness levels
//  Sub-frame runs every SUB_MS ms → 4×3 = 12ms cycle = ~83Hz flicker-free
const int BCM_LEVELS = 8;
const int SUB_MS     = 2;

inline int heightToBrightness(float v) {
  if (v < 0.10f) return 0;
  if (v < 0.30f) return 1;
  if (v < 0.55f) return 2;
  if (v < 0.75f) return 3;
  if (v < 0.95f) return 4;
  if (v < 1.10f) return 5;
  if (v < 1.25f) return 6;
  if (v < 1.38f) return 7;
  return 8;
}

// ══════════════════════════════════════════════════════
// CORE 1 - Physics task
// ══════════════════════════════════════════════════════
float prevAx = 0, prevAy = 0;
float localH[N][N], localVel[N][N];

void physicsTask(void* param) {
  for (;;) {
    // Read latest accelerometer values
    float ax, ay;
    portENTER_CRITICAL(&accelMux);
      ax = shared_ax;
      ay = shared_ay;
    portEXIT_CRITICAL(&accelMux);

    // Jerk = change in acceleration → only motion events drive the wave
    // A still board on a tilted surface → jx/jy = 0 → wave stays calm
    float jx = ax - prevAx;
    float jy = ay - prevAy;
    prevAx = ax;
    prevAy = ay;

    // Copy shared state into local buffers
    portENTER_CRITICAL(&waveMux);
      memcpy(localH,   (const void*)h,   sizeof(localH));
      memcpy(localVel, (const void*)vel, sizeof(localVel));
    portEXIT_CRITICAL(&waveMux);

    float hNext[N][N];

    for (int r = 0; r < N; r++) {
      for (int c = 0; c < N; c++) {
        // Reflective boundary - waves bounce off walls
        float up = (r > 0)   ? localH[r-1][c] : localH[r][c];
        float dn = (r < N-1) ? localH[r+1][c] : localH[r][c];
        float lt = (c > 0)   ? localH[r][c-1] : localH[r][c];
        float rt = (c < N-1) ? localH[r][c+1] : localH[r][c];

        // 2D wave equation: acceleration = c² × laplacian
        float lap = up + dn + lt + rt - 4.0f * localH[r][c];

        // Project jerk onto grid position for directional kick
        float nx    = (c - 3.5f) / 3.5f;
        float ny    = (r - 3.5f) / 3.5f;
        float force = (jx * nx + jy * ny) * FORCE;

        localVel[r][c] += C2 * lap - DAMPING * localVel[r][c] + force;
        hNext[r][c] = constrain(localH[r][c] + localVel[r][c], -1.5f, 1.5f);
      }
    }

    // Write results back to shared state
    portENTER_CRITICAL(&waveMux);
      memcpy((void*)h,   hNext,    sizeof(hNext));
      memcpy((void*)vel, localVel, sizeof(localVel));
    portEXIT_CRITICAL(&waveMux);

    // Compute total wave energy (sum of height² + velocity²)
    float energy = 0.0f;
    for (int r = 0; r < N; r++)
      for (int c = 0; c < N; c++)
        energy += hNext[r][c] * hNext[r][c] + localVel[r][c] * localVel[r][c];

    portENTER_CRITICAL(&energyMux);
      shared_energy = energy;
    portEXIT_CRITICAL(&energyMux);

    vTaskDelay(pdMS_TO_TICKS(16));  // ~60fps physics
  }
}

// ══════════════════════════════════════════════════════
// CORE 0 - Render + IMU task
// ══════════════════════════════════════════════════════
void renderTask(void* param) {
  int subFrame = 0;
  static int bright[N][N];

  for (;;) {
    // Read IMU every cycle
    mpu.update();
    portENTER_CRITICAL(&accelMux);
      shared_ax = mpu.getAccX();
      shared_ay = mpu.getAccY();
    portEXIT_CRITICAL(&accelMux);

    // Read energy and map to hardware intensity 0-15
    float energy;
    portENTER_CRITICAL(&energyMux);
      energy = shared_energy;
    portEXIT_CRITICAL(&energyMux);

    // Max possible energy on an 8×8 grid ≈ 64 × (1.5²+1.5²) = 288
    // Map 0–288 → intensity 0–15 with a sqrt curve so dim end is more visible
    float normalized = sqrtf(constrain(energy / 260.0f, 0.0f, 1.0f));
    int intensity = (int)(normalized * 15.0f);
    intensity = constrain(intensity, 0, 15);
    lc.control(MD_MAX72XX::INTENSITY, intensity);

    // Snapshot wave heights
    float snap[N][N];
    portENTER_CRITICAL(&waveMux);
      memcpy(snap, (const void*)h, sizeof(snap));
    portEXIT_CRITICAL(&waveMux);

    // Recompute brightness levels once per BCM cycle (subFrame 0)
    if (subFrame == 0) {
      for (int r = 0; r < N; r++)
        for (int c = 0; c < N; c++)
          bright[r][c] = heightToBrightness(snap[r][c]);
    }

    // Push one BCM sub-frame: LED ON if brightness_level > subFrame index
    for (int r = 0; r < N; r++) {
      byte row = 0;
      for (int c = 0; c < N; c++)
        if (bright[r][c] > subFrame) row |= (1 << (7 - c));
      lc.setRow(0, r, row);
    }

    subFrame = (subFrame + 1) % BCM_LEVELS;
    vTaskDelay(pdMS_TO_TICKS(SUB_MS));  // 3ms per sub-frame
  }
}

// ══════════════════════════════════════════════════════
// SETUP
// ══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // Init MPU6050
  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("MPU6050 not found! Check wiring.");
    while (true);   // halt - nothing will work without the IMU
  }

  Serial.println("Hold still — calibrating MPU6050 (1 sec)...");
  delay(1000);
  mpu.calcOffsets();  // zeroes out gravity so still board = zero force
  Serial.println("Calibration done. Wave engine starting.");

  // Init MAX7219
  lc.begin();
  lc.control(MD_MAX72XX::INTENSITY, 10);
  lc.clear();

  // Launch tasks on separate cores
  // renderTask gets priority 2, so BCM timing stays tight
  xTaskCreatePinnedToCore(physicsTask, "Physics", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(renderTask,  "Render",  4096, NULL, 2, NULL, 0);
}

void loop() {
  vTaskDelete(NULL);  // main loop unused - FreeRTOS takes over
}
