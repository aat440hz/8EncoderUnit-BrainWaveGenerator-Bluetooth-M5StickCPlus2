/*
  8EncoderUnit-BrainWaveGenerator-M5StickCPlus2 — Bluetooth Speaker Edition
  ---------------------------------------------------------------------------
  Generates two mixed sine tones with an amplitude-modulated "pulse" — the
  same math the original web-page version used (two oscillators summed,
  gain modulated by a third LFO) — and streams the audio directly over
  classic Bluetooth A2DP to a Bluetooth speaker. No WiFi, no web page, no
  browser required.

  Hardware: M5StickC PLUS2 + M5Stack 8Encoder Unit (I2C, pins 32/33)
  The M5StickC PLUS2 is built on the ESP32-PICO-V3-02, a classic ESP32
  variant with Bluetooth Classic + BLE, so A2DP source mode works fine.

  Required library (install via Arduino Library Manager):
    "ESP32-A2DP" by Phil Schatzmann (github.com/pschatzmann/ESP32-A2DP)

  How it works:
    1. On boot: if a speaker was connected before, the device pages it
       directly (a real reconnect attempt, not an inquiry scan — see the
       reconnect note below) for up to ~25s. Otherwise, or if you press
       BtnB to skip, it scans for nearby speakers/receivers:
         - Press the side button (BtnB) to move the cursor to the next
           discovered speaker.
         - Press the front button (BtnA) to connect to the highlighted
           speaker.
    2. Once connected, the 8 encoder knobs control the tone exactly like
       the original version:
         Encoder 0/1/2 -> Frequency 1 (base / fine / multiplier)
         Encoder 4/5/6 -> Frequency 2 (base / fine / multiplier)
         Encoder 3/7   -> Pulse rate (base / fine)
       Press BtnA to start/stop playback.
       Hold BtnB for ~1.5s to disconnect, forget this speaker, and go
       pick a different one.

  Notes / assumptions (things to check if it doesn't compile as-is,
  since library APIs can shift slightly between versions):
    - is_connected() is used to detect a successful pairing. If your
      installed ESP32-A2DP version doesn't expose that method, swap it
      for `a2dp_source.is_active(0)` or hook
      `set_on_connection_state_changed()` instead.
    - set_auto_reconnect(esp_bd_addr_t, int) is used to page the
      remembered speaker directly instead of doing an inquiry scan for
      it. Most speakers stop responding to inquiry scans once they're
      out of pairing mode (they only listen for a direct connect from a
      device they already know), so a plain inquiry-based reconnect can
      fail even though the speaker is right there and willing to accept
      a direct connection — this is what a direct page fixes. If your
      installed ESP32-A2DP version doesn't have this exact overload,
      check its BluetoothA2DPSource.h for the closest equivalent
      (connect_to(addr) plus set_auto_reconnect(true, retries) is the
      same idea split into two calls).

  Performance note: the audio callback uses a precomputed sine lookup
  table instead of calling sin()/sinf() per sample. The ESP32 only has a
  single-precision hardware FPU (no hardware double), so calling sin() on
  a `double` at 44.1kHz x 3 oscillators falls back to slow software float
  emulation — the synth can't keep up with real time, which sounds like
  periodic skips and can starve the core running the Bluetooth stack
  badly enough to trip the watchdog timer and reboot the device. The
  lookup table avoids trig calls in the hot path entirely.

  Drift: every 5 seconds each tone gets a fresh random +/-10Hz offset
  (and the pulse gets a smaller +/-0.15Hz wobble — pulse rate matters a
  lot for entrainment, so it stays close to what you dialed in), applied
  on top of whatever the knobs are set to. This is an abrupt re-roll, not
  a smooth glide — same as the original web-page version — which is what
  gives it the slightly-off, alien character. The on-screen numbers
  always show the knob-set base values; drift only affects what comes
  out of the speaker.

  Speaker memory: the last speaker you connected to is saved to flash
  (NVS) and paged directly on boot (~25s timeout, then falls back to the
  normal pick screen). Hold BtnB while running to disconnect AND forget
  it, so next boot goes straight to picking a new one.

  Why reconnect failed after using the physical power button: that
  button cuts power at the PMIC with no graceful shutdown hook available
  to this sketch — that part can't be fixed in firmware. But the actual
  bug was the reconnect logic itself relying on an inquiry scan, which
  doesn't find most already-paired speakers regardless of how the
  previous session ended. Direct paging (above) fixes that independent
  of whether the shutdown was clean.

  Volume quieter after reconnect than after a fresh pairing: this comes
  from the AVRCP side-channel, not the audio stream itself. The
  ESP32-A2DP library acts as an AVRCP controller and listens for the
  speaker to report "volume changed" notifications; whenever it gets
  one, it quietly calls its own set_volume() with whatever number the
  speaker reported, which scales all outgoing audio (see
  BluetoothA2DPSource::avrc_ct_evt_handler, case
  ESP_AVRC_RN_VOLUME_CHANGE, in the library source). On a brand-new
  pairing the speaker tends to report a fresh/high volume state right
  away; on a reconnect it can report back whatever lower volume it last
  remembered (or a stale cached value), and that silently overrides our
  audio gain a moment after the connection looks established. The fix
  is to not trust that: onConnected() forces set_volume() to max, and
  loopRunning() keeps re-asserting it for a few seconds after connecting
  so it wins even if the speaker's notification arrives slightly late.

  Battery: every screen's header bar shows the remaining battery percent
  in the top-right corner, read from M5.Power (green above 50%, amber
  20-50%, red below that). If your board package doesn't expose
  M5.Power.getBatteryLevel(), swap it for the older M5.Axp.GetBatteryLevel()
  call in drawBatteryBadge().
*/

#include <M5StickCPlus2.h>
#include "UNIT_8ENCODER.h"
#include <Wire.h>
#include <string.h>
#include <Preferences.h>
#include "esp_gap_bt_api.h"
#include "BluetoothA2DPSource.h"

UNIT_8ENCODER sensor;
BluetoothA2DPSource a2dp_source;

// Off-screen draw buffer so redraws are pushed to the display in one
// shot instead of being visible mid-draw (which is what caused the
// flashing). If your board package doesn't have M5Canvas, use
// M5.Lcd.createSprite(w,h) / M5.Lcd.pushSprite(0,0) instead (older
// TFT_eSprite-style API) — same idea, different class name.
M5Canvas canvas(&M5.Lcd);

// Persists the last-connected speaker's address/name across reboots.
Preferences prefs;

// ---------- App state ----------
enum AppState { STATE_AUTO_RECONNECT, STATE_SCANNING, STATE_CONNECTING, STATE_RUNNING };
AppState appState = STATE_SCANNING;

// ---------- Bluetooth device scan/select ----------
#define MAX_DEVICES 8
struct FoundDevice {
  char name[32];
  esp_bd_addr_t addr;
  int rssi;
};
FoundDevice devices[MAX_DEVICES];
int deviceCount = 0;
int selectedIndex = 0;

bool connectRequested = false;
esp_bd_addr_t targetAddr;
unsigned long autoReconnectStartTime = 0;

// Identity of whichever speaker we're connecting/connected to right now,
// independent of the scan list indices above (the auto-reconnect path
// doesn't go through the picker at all).
char currentSpeakerName[32] = "";
esp_bd_addr_t currentSpeakerAddr;

unsigned long btnBHeldSince = 0;
bool btnBHeldLongHandled = false;

// ---------- Volume ----------
// The source-side software gain (0-127, see onConnected()). Forced to max
// on every connect and re-asserted for a few seconds afterward, because
// the speaker's own remembered AVRCP volume can otherwise arrive a moment
// later and quietly override it (that's the "quieter after reconnect"
// bug — see the note near the top of this file). 0 means "not currently
// in the post-connect settle window."
#define TARGET_VOLUME 127
unsigned long connectionSettleUntil = 0;

// ---------- Audio (shared with the A2DP data callback) ----------
volatile bool isPlaying = false;
volatile float currentFreq1 = 0.0f;
volatile float currentFreq2 = 0.0f;
volatile float currentPulseRate = 0.0f;

float lastFreq1 = -1.0f;
float lastFreq2 = -1.0f;
float lastPulseRate = -1.0f;

int encoderZeroValue1 = 0;
int encoderZeroValue2 = 0;

// ---------- Knob tuning ----------
// Some 8Encoder units report more than 1 raw count per physical detent
// click. If turning a knob exactly one click moves the on-screen value
// by more than you'd expect (e.g. the pulse fine knob, channel 7, jumps
// by 0.2Hz instead of 0.1Hz — making values like 2.5 or 4.5 impossible
// to land on, since they'd sit "between" two reachable positions), raise
// this to match: try 2, re-test one click, and adjust from there.
#define ENCODER_COUNTS_PER_CLICK 1

// The multiplier knob (channel 2 for Freq 1, channel 6 for Freq 2)
// originally multiplied the whole frequency by (1 + clicks) — i.e. every
// single click doubled/tripled/quadrupled it. MULTIPLIER_STEP is how
// much each click adds instead (0.25 = +25% per click). Lower it for
// gentler steps, raise it for the old aggressive-jump behavior.
#define MULTIPLIER_STEP 0.25f

// ---------- Sine lookup table (avoids per-sample trig calls) ----------
#define SINE_TABLE_SIZE 1024
float sineTable[SINE_TABLE_SIZE];

// ---------- UI palette (set in setup() once the canvas exists) ----------
uint16_t colorBg;
uint16_t colorPanel;
uint16_t colorAccent;
uint16_t colorText;
uint16_t colorDim;
uint16_t colorPlaying;
uint16_t colorStopped;
uint16_t colorHighlightBg;
uint16_t colorBattLow;

// ---------- Function prototypes ----------
float calculateFrequency(int baseValue, int fineTuneValue, int multiplierValue);
float calculatePulseRate(int baseValue, int fineAdjustment);
void updateRunningDisplay(float f1, float f2, float pulse);
void drawValueRow(int y, const char* label, float value);
void drawConnectingScreen(const char* title, const char* name, const char* hint, float progress);
int drawBatteryBadge(int rightEdgeX, int y);
void updateScanDisplay();
void loopAutoReconnect();
void loopScanning();
void loopConnecting();
void loopRunning();
void onConnected();
void zeroEncoders();
int adjustedEncoderValue1(int rawValue);
int adjustedEncoderValue2(int rawValue);
bool ssidCallback(const char* ssid, esp_bd_addr_t address, int rssi);
int32_t audioDataCallback(Frame* frame, int32_t frameCount);

void setup() {
    M5.begin();
    Wire.begin(32, 33);
    sensor.begin();

    M5.Lcd.setRotation(1);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(2);

    canvas.createSprite(M5.Lcd.width(), M5.Lcd.height());
    canvas.setTextColor(WHITE);
    canvas.setTextSize(2);

    colorBg          = canvas.color565(8, 10, 18);     // near-black navy
    colorPanel       = canvas.color565(28, 33, 48);    // header/divider panel
    colorAccent      = canvas.color565(0, 200, 210);   // teal accent / values
    colorText        = canvas.color565(235, 240, 245); // off-white
    colorDim         = canvas.color565(120, 130, 145); // muted grey-blue
    colorPlaying     = canvas.color565(60, 200, 130);  // green status bar
    colorStopped     = canvas.color565(220, 130, 60);  // amber status bar
    colorHighlightBg = canvas.color565(0, 60, 65);     // selected row background
    colorBattLow     = canvas.color565(220, 60, 60);   // low-battery red

    zeroEncoders();
    randomSeed(micros());  // seeds the pitch/pulse drift's random rerolls

    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        sineTable[i] = sinf(2.0f * PI * (float)i / (float)SINE_TABLE_SIZE);
    }

    // Look up the last speaker we successfully connected to, if any.
    prefs.begin("btspk", false);
    bool haveSaved = false;
    size_t addrLen = prefs.getBytes("addr", currentSpeakerAddr, sizeof(currentSpeakerAddr));
    if (addrLen == sizeof(currentSpeakerAddr)) {
        String nm = prefs.getString("name", "");
        if (nm.length() > 0) {
            nm.toCharArray(currentSpeakerName, sizeof(currentSpeakerName));
            haveSaved = true;
        }
    }

    a2dp_source.set_data_callback_in_frames(audioDataCallback);
    a2dp_source.set_ssid_callback(ssidCallback);

    if (haveSaved) {
        // Direct reconnect to the known address (a "page"), not an
        // inquiry scan. A speaker that's already been paired normally
        // stops responding to inquiry scans once it's out of pairing
        // mode, but it still listens for a direct connect from a device
        // it already knows — this is what actually lets it reconnect
        // after the previous session ended abruptly (e.g. the physical
        // power button, which cuts power without a graceful disconnect).
        a2dp_source.set_auto_reconnect(currentSpeakerAddr, 5);
        autoReconnectStartTime = millis();
        appState = STATE_AUTO_RECONNECT;
    } else {
        a2dp_source.set_auto_reconnect(false);
        appState = STATE_SCANNING;
    }

    // start() attempts the direct reconnect above first (if configured),
    // then falls back to open discovery governed by ssidCallback — which
    // is also how the manual picker's device list gets populated.
    a2dp_source.start();
}

void loop() {
    M5.update();

    switch (appState) {
        case STATE_AUTO_RECONNECT:
            loopAutoReconnect();
            break;
        case STATE_SCANNING:
            loopScanning();
            break;
        case STATE_CONNECTING:
            loopConnecting();
            break;
        case STATE_RUNNING:
            loopRunning();
            break;
    }
}

// ---------------------------------------------------------------------
// Scanning / selecting a speaker
// ---------------------------------------------------------------------

bool ssidCallback(const char* ssid, esp_bd_addr_t address, int rssi) {
    if (ssid == nullptr || strlen(ssid) == 0) return false;

    int idx = -1;
    for (int i = 0; i < deviceCount; i++) {
        if (memcmp(devices[i].addr, address, ESP_BD_ADDR_LEN) == 0) {
            idx = i;
            break;
        }
    }
    // Some speakers advertise the same friendly name from more than one
    // internal BT address, which looked like duplicate entries in the
    // list. Fall back to matching by name so they collapse into one row.
    if (idx < 0) {
        for (int i = 0; i < deviceCount; i++) {
            if (strncmp(devices[i].name, ssid, sizeof(devices[i].name) - 1) == 0) {
                idx = i;
                break;
            }
        }
    }
    if (idx < 0 && deviceCount < MAX_DEVICES) {
        idx = deviceCount++;
        memcpy(devices[idx].addr, address, ESP_BD_ADDR_LEN);
    }
    if (idx >= 0) {
        strncpy(devices[idx].name, ssid, sizeof(devices[idx].name) - 1);
        devices[idx].name[sizeof(devices[idx].name) - 1] = '\0';
        devices[idx].rssi = rssi;
        // Note: addr is intentionally left as whichever address was
        // first seen for this name/slot, so the connect target below
        // doesn't shift underneath the user once they've picked a row.
    }

    if (connectRequested && idx >= 0 &&
        memcmp(address, targetAddr, ESP_BD_ADDR_LEN) == 0) {
        return true;  // lock on to this device -> library connects to it
    }
    return false;  // keep scanning
}

// ---------------------------------------------------------------------
// Auto-reconnect to the last speaker we successfully connected to
// ---------------------------------------------------------------------

// Draws the battery percentage right-aligned to rightEdgeX (color-coded:
// green above 50%, amber 20-50%, red below that) and returns the pixel
// width it used, including a small left margin, so a caller can
// right-align other header text to sit just to its left without
// overlapping it. M5.Power comes from the M5Unified base this board
// package builds on; if yours doesn't expose it, swap the call below for
// the older M5.Axp.GetBatteryLevel().
int drawBatteryBadge(int rightEdgeX, int y) {
    int level = M5.Power.getBatteryLevel();  // 0-100, or -1 if unknown
    char buf[8];
    uint16_t col;
    if (level < 0) {
        snprintf(buf, sizeof(buf), "--%%");
        col = colorDim;
    } else {
        if (level > 50) col = colorPlaying;
        else if (level > 20) col = colorStopped;
        else col = colorBattLow;
        snprintf(buf, sizeof(buf), "%d%%", level);
    }
    canvas.setTextColor(col);
    canvas.setTextSize(1);
    int w = canvas.textWidth(buf);
    canvas.setCursor(rightEdgeX - w, y);
    canvas.print(buf);
    return w + 8;
}

// Shared "connecting" look for both the auto-reconnect and manual-pick
// paths. progress is 0..1 to draw a progress bar, or negative to hide it.
void drawConnectingScreen(const char* title, const char* name, const char* hint, float progress) {
    canvas.fillSprite(colorBg);

    canvas.fillRect(0, 0, canvas.width(), 14, colorPanel);
    canvas.setTextColor(colorDim);
    canvas.setTextSize(1);
    canvas.setCursor(6, 4);
    canvas.print(title);

    drawBatteryBadge(canvas.width() - 6, 4);

    canvas.setTextColor(colorText);
    canvas.setTextSize(2);
    canvas.setCursor(8, 40);
    canvas.print(name);

    if (progress >= 0.0f) {
        if (progress > 1.0f) progress = 1.0f;
        int barX = 8, barY = 72, barW = canvas.width() - 16, barH = 6;
        canvas.drawRect(barX, barY, barW, barH, colorDim);
        canvas.fillRect(barX + 1, barY + 1, (int)((barW - 2) * progress), barH - 2, colorAccent);
    }

    canvas.drawFastHLine(0, canvas.height() - 12, canvas.width(), colorPanel);
    canvas.setTextColor(colorDim);
    canvas.setTextSize(1);
    canvas.setCursor(4, canvas.height() - 9);
    canvas.print(hint);

    canvas.pushSprite(0, 0);
}

// Direct-connect retries take longer than an inquiry match did (each
// retry is a real page attempt), so this window is more generous than
// the old inquiry-based approach needed.
#define AUTO_RECONNECT_TIMEOUT_MS 25000

void loopAutoReconnect() {
    static unsigned long lastRefresh = 0;
    if (millis() - lastRefresh > 150) {
        float progress = (float)(millis() - autoReconnectStartTime) / (float)AUTO_RECONNECT_TIMEOUT_MS;
        drawConnectingScreen("RECONNECTING", currentSpeakerName, "B: PICK A DIFFERENT SPEAKER", progress);
        lastRefresh = millis();
    }

    if (a2dp_source.is_connected()) {
        onConnected();
        return;
    }

    bool timedOut = millis() - autoReconnectStartTime > AUTO_RECONNECT_TIMEOUT_MS;
    // Just skips to the picker for this boot — doesn't forget the saved
    // speaker. Hold BtnB once running (see loopRunning) to forget it.
    if (timedOut || M5.BtnB.wasPressed()) {
        // Stop the background retry attempts so they don't race with
        // whatever the user picks manually from here on.
        a2dp_source.set_auto_reconnect(false);
        appState = STATE_SCANNING;
    }
}

// ---------------------------------------------------------------------
// Scanning / selecting a speaker
// ---------------------------------------------------------------------

void loopScanning() {
    // Covers the rare case where a background auto-reconnect retry
    // (still in flight from before we gave up and switched to manual
    // picking) succeeds late — jump straight into the running screen
    // instead of leaving the picker up while already connected.
    if (a2dp_source.is_connected()) {
        onConnected();
        return;
    }

    bool changed = false;

    if (M5.BtnB.wasPressed() && deviceCount > 0) {
        selectedIndex = (selectedIndex + 1) % deviceCount;
        changed = true;
    }

    if (M5.BtnA.wasPressed() && deviceCount > 0) {
        strncpy(currentSpeakerName, devices[selectedIndex].name, sizeof(currentSpeakerName) - 1);
        currentSpeakerName[sizeof(currentSpeakerName) - 1] = '\0';
        memcpy(currentSpeakerAddr, devices[selectedIndex].addr, ESP_BD_ADDR_LEN);
        memcpy(targetAddr, currentSpeakerAddr, ESP_BD_ADDR_LEN);
        connectRequested = true;
        appState = STATE_CONNECTING;
        changed = true;
    }

    static unsigned long lastRefresh = 0;
    if (changed || millis() - lastRefresh > 500) {
        updateScanDisplay();
        lastRefresh = millis();
    }
}

void updateScanDisplay() {
    canvas.fillSprite(colorBg);

    canvas.fillRect(0, 0, canvas.width(), 14, colorPanel);
    canvas.setTextColor(colorDim);
    canvas.setTextSize(1);
    canvas.setCursor(6, 4);
    canvas.print("FIND SPEAKER");

    int battW = drawBatteryBadge(canvas.width() - 6, 4);

    char countBuf[14];
    snprintf(countBuf, sizeof(countBuf), "%d found", deviceCount);
    canvas.setTextColor(colorDim);
    canvas.setTextSize(1);
    canvas.setCursor(canvas.width() - battW - canvas.textWidth(countBuf) - 6, 4);
    canvas.print(countBuf);

    int y = 18;
    int rowH = 14;
    if (deviceCount == 0) {
        canvas.setTextColor(colorDim);
        canvas.setTextSize(1);
        canvas.setCursor(6, y + 4);
        canvas.print("Scanning");
        int dots = (millis() / 400) % 4;
        for (int d = 0; d < dots; d++) canvas.print(".");
    } else {
        for (int i = 0; i < deviceCount; i++) {
            int rowY = y + i * rowH;
            if (i == selectedIndex) {
                canvas.fillRect(2, rowY, canvas.width() - 4, rowH - 1, colorHighlightBg);
                canvas.fillRect(2, rowY, 3, rowH - 1, colorAccent);
                canvas.setTextColor(colorText);
            } else {
                canvas.setTextColor(colorDim);
            }
            canvas.setTextSize(1);
            canvas.setCursor(10, rowY + 3);
            canvas.print(devices[i].name);
        }
    }

    canvas.drawFastHLine(0, canvas.height() - 12, canvas.width(), colorPanel);
    canvas.setTextColor(colorDim);
    canvas.setTextSize(1);
    canvas.setCursor(4, canvas.height() - 9);
    canvas.print("B: NEXT      A: CONNECT");

    canvas.pushSprite(0, 0);
}

void loopConnecting() {
    static unsigned long lastRefresh = 0;
    if (millis() - lastRefresh > 150) {
        drawConnectingScreen("CONNECTING", currentSpeakerName, "PLEASE WAIT", -1.0f);
        lastRefresh = millis();
    }

    if (a2dp_source.is_connected()) {
        onConnected();
    }
}

// Shared by the auto-reconnect and manual-pick paths once the library
// reports a successful connection.
void onConnected() {
    // Remember this speaker so we auto-reconnect to it next boot.
    prefs.putBytes("addr", currentSpeakerAddr, ESP_BD_ADDR_LEN);
    prefs.putString("name", currentSpeakerName);

    // Force our own audio gain to max right away, then keep re-asserting
    // it for a few seconds (see loopRunning()) in case the speaker's own
    // remembered AVRCP volume shows up a moment later and overrides this
    // single call — that race is what made reconnects sound quieter than
    // a fresh pairing.
    a2dp_source.set_volume(TARGET_VOLUME);
    connectionSettleUntil = millis() + 4000;

    appState = STATE_RUNNING;
    zeroEncoders();
    lastFreq1 = lastFreq2 = lastPulseRate = -1.0f;
    canvas.fillSprite(colorBg);
    canvas.pushSprite(0, 0);
}

// ---------------------------------------------------------------------
// Running: encoders control tone, BtnA toggles play/stop
// ---------------------------------------------------------------------

void loopRunning() {
    // Re-assert full volume for a few seconds after connecting. See
    // onConnected(): the speaker can report its own remembered AVRCP
    // volume a moment after the connection looks established, which
    // would otherwise silently override the gain we just set.
    if (connectionSettleUntil != 0) {
        static unsigned long lastVolumeReassert = 0;
        if (millis() - lastVolumeReassert > 400) {
            a2dp_source.set_volume(TARGET_VOLUME);
            lastVolumeReassert = millis();
        }
        if (millis() > connectionSettleUntil) {
            connectionSettleUntil = 0;
        }
    }

    int rawEncoderValue1_0 = sensor.getEncoderValue(0);
    int rawEncoderValue1_1 = sensor.getEncoderValue(1);
    int rawEncoderValue1_2 = sensor.getEncoderValue(2);

    int rawEncoderValue2_4 = sensor.getEncoderValue(4);
    int rawEncoderValue2_5 = sensor.getEncoderValue(5);
    int rawEncoderValue2_6 = sensor.getEncoderValue(6);

    float freq1 = calculateFrequency(adjustedEncoderValue1(rawEncoderValue1_0),
                                      adjustedEncoderValue1(rawEncoderValue1_1),
                                      adjustedEncoderValue1(rawEncoderValue1_2));
    float freq2 = calculateFrequency(adjustedEncoderValue2(rawEncoderValue2_4),
                                      adjustedEncoderValue2(rawEncoderValue2_5),
                                      adjustedEncoderValue2(rawEncoderValue2_6));

    float pulseRate = calculatePulseRate(sensor.getEncoderValue(3) / ENCODER_COUNTS_PER_CLICK,
                                          sensor.getEncoderValue(7) / ENCODER_COUNTS_PER_CLICK);

    // Alien drift: every 5s each tone gets a fresh random offset (an
    // abrupt re-roll, not a glide) layered on top of the knob-set base
    // values, plus a smaller wobble on the pulse. Only the audio hears
    // this — the display below always shows the clean base values.
    static unsigned long lastDriftUpdate = 0;
    static float driftOffset1 = 0.0f;
    static float driftOffset2 = 0.0f;
    static float driftOffsetPulse = 0.0f;
    const unsigned long driftIntervalMs = 5000;
    if (millis() - lastDriftUpdate > driftIntervalMs) {
        driftOffset1 = random(-1000, 1001) / 100.0f;      // -10.00 .. +10.00 Hz
        driftOffset2 = random(-1000, 1001) / 100.0f;      // -10.00 .. +10.00 Hz
        driftOffsetPulse = random(-15, 16) / 100.0f;      // -0.15 .. +0.15 Hz
        lastDriftUpdate = millis();
    }

    float effectivePulseRate;
    if (pulseRate > 0.0f) {
        effectivePulseRate = pulseRate + driftOffsetPulse;
        if (effectivePulseRate < 0.1f) effectivePulseRate = 0.1f;
    } else {
        // Pulse rate 0 means "no pulse" — keep it exactly steady rather
        // than drifting it across the on/off boundary.
        effectivePulseRate = 0.0f;
    }

    // These are what the audio callback actually reads from.
    currentFreq1 = freq1 + driftOffset1;
    currentFreq2 = freq2 + driftOffset2;
    currentPulseRate = effectivePulseRate;

    // Throttled so rapid knob turns don't flood the SPI display with
    // full-screen redraws and steal CPU time from audio generation.
    static unsigned long lastDisplayUpdate = 0;
    if ((freq1 != lastFreq1 || freq2 != lastFreq2 || pulseRate != lastPulseRate) &&
        millis() - lastDisplayUpdate > 100) {
        updateRunningDisplay(freq1, freq2, pulseRate);
        lastFreq1 = freq1;
        lastFreq2 = freq2;
        lastPulseRate = pulseRate;
        lastDisplayUpdate = millis();
    }

    if (M5.BtnA.wasPressed()) {
        isPlaying = !isPlaying;
        updateRunningDisplay(freq1, freq2, pulseRate);
    }

    // Hold BtnB ~1.5s to disconnect, forget this speaker, and go pick a
    // different one (next boot won't try to auto-reconnect to it).
    if (M5.BtnB.isPressed()) {
        if (btnBHeldSince == 0) btnBHeldSince = millis();
        if (!btnBHeldLongHandled && millis() - btnBHeldSince > 1500) {
            btnBHeldLongHandled = true;
            a2dp_source.set_auto_reconnect(false);
            a2dp_source.disconnect();  // proper AVDTP disconnect, not just a reboot
            prefs.remove("addr");
            prefs.remove("name");
            ESP.restart();
        }
    } else {
        btnBHeldSince = 0;
        btnBHeldLongHandled = false;
    }
}

// One labeled "instrument panel" row: a small dim label, a big value in
// the accent color, and a dim unit suffix placed right after it.
void drawValueRow(int y, const char* label, float value) {
    canvas.setTextColor(colorDim);
    canvas.setTextSize(1);
    canvas.setCursor(8, y);
    canvas.print(label);

    canvas.setTextColor(colorAccent);
    canvas.setTextSize(3);
    canvas.setCursor(8, y + 9);
    canvas.printf("%.2f", value);

    canvas.setTextColor(colorDim);
    canvas.setTextSize(1);
    canvas.setCursor(canvas.getCursorX() + 4, y + 22);
    canvas.print("Hz");
}

void updateRunningDisplay(float f1, float f2, float pulse) {
    canvas.fillSprite(colorBg);

    canvas.fillRect(0, 0, canvas.width(), 14, colorPanel);
    canvas.setTextColor(colorDim);
    canvas.setTextSize(1);
    canvas.setCursor(6, 4);
    canvas.print("BRAINWAVE GENERATOR");

    drawBatteryBadge(canvas.width() - 6, 4);

    drawValueRow(16, "FREQ 1", f1);
    drawValueRow(48, "FREQ 2", f2);
    drawValueRow(80, "PULSE", pulse);

    // Footer: a full-width color-coded status bar so play/stop state is
    // glanceable, plus the connected speaker name.
    int footerY = 112;
    uint16_t statusColor = isPlaying ? colorPlaying : colorStopped;
    canvas.fillRect(0, footerY, canvas.width(), canvas.height() - footerY, statusColor);
    canvas.setTextColor(colorBg);
    canvas.setTextSize(1);
    canvas.setCursor(6, footerY + 4);
    canvas.print(isPlaying ? "> PLAYING" : "STOPPED");

    char nameBuf[20];
    strncpy(nameBuf, currentSpeakerName, sizeof(nameBuf) - 1);
    nameBuf[sizeof(nameBuf) - 1] = '\0';
    canvas.setCursor(canvas.width() - canvas.textWidth(nameBuf) - 6, footerY + 4);
    canvas.print(nameBuf);

    canvas.pushSprite(0, 0);
}

// ---------------------------------------------------------------------
// Audio synthesis — runs on the Bluetooth stack's task; keep it fast
// and allocation-free.
// ---------------------------------------------------------------------

int32_t audioDataCallback(Frame* frame, int32_t frameCount) {
    // Phases are kept in table-index units (0 .. SINE_TABLE_SIZE) rather
    // than radians, so no radian<->index conversion is needed per sample.
    static float phase1 = 0.0f;
    static float phase2 = 0.0f;
    static float phasePulse = 0.0f;
    const float sampleRate = 44100.0f;

    if (!isPlaying) {
        for (int i = 0; i < frameCount; i++) {
            frame[i].channel1 = 0;
            frame[i].channel2 = 0;
        }
        return frameCount;
    }

    // Snapshot the shared control values once per callback.
    float f1 = currentFreq1;
    float f2 = currentFreq2;
    float pr = currentPulseRate;
    const float amp = 8000.0f;  // per-oscillator headroom (out of 32767)

    float inc1 = f1 * (float)SINE_TABLE_SIZE / sampleRate;
    float inc2 = f2 * (float)SINE_TABLE_SIZE / sampleRate;
    float incPulse = pr * (float)SINE_TABLE_SIZE / sampleRate;
    // At pulse rate 0 the LFO phase never advances, so it sits frozen at
    // whatever point it happened to stop on — reading that back gave a
    // fixed but effectively random gain (quiet some times, loud others).
    // Treat "no pulse" as "no modulation": hold gain at a steady 1.0.
    bool pulseActive = pr > 0.0f;

    for (int i = 0; i < frameCount; i++) {
        float s1 = sineTable[(int)phase1];
        float s2 = sineTable[(int)phase2];
        // Same trick the original web-audio version used: an LFO added to
        // a base gain of 1 makes the combined tone swell 0x -> 2x at the
        // pulse rate, i.e. an isochronic-style pulsing beat.
        float gainEnv = pulseActive ? (1.0f + sineTable[(int)phasePulse]) : 1.0f;

        float mixed = (s1 + s2) * amp * gainEnv * 0.5f;
        if (mixed > 32000.0f) mixed = 32000.0f;
        if (mixed < -32000.0f) mixed = -32000.0f;

        int16_t sample = (int16_t)mixed;
        frame[i].channel1 = sample;
        frame[i].channel2 = sample;

        // fmodf (not a plain conditional subtract) so wrapping stays
        // correct even if a knob pushes a frequency high enough that the
        // per-sample increment exceeds the table size.
        phase1 = fmodf(phase1 + inc1, (float)SINE_TABLE_SIZE);
        phase2 = fmodf(phase2 + inc2, (float)SINE_TABLE_SIZE);
        phasePulse = fmodf(phasePulse + incPulse, (float)SINE_TABLE_SIZE);
    }
    return frameCount;
}

// ---------------------------------------------------------------------
// Helpers carried over from the original sketch
// ---------------------------------------------------------------------

float calculateFrequency(int baseValue, int fineTuneValue, int multiplierValue) {
    if (baseValue < 0) baseValue = 0;
    if (fineTuneValue < 0) fineTuneValue = 0;
    if (multiplierValue < 0) multiplierValue = 0;

    float frequency = 20.0 + baseValue * 2.0;
    frequency += fineTuneValue * 0.1;
    frequency *= (1.0f + multiplierValue * MULTIPLIER_STEP);
    return frequency;
}

float calculatePulseRate(int baseValue, int fineAdjustment) {
    if (baseValue < 0) baseValue = 0;
    if (fineAdjustment < 0) fineAdjustment = 0;

    float pulseRate = baseValue * 1.0;
    pulseRate += fineAdjustment * 0.1;
    return pulseRate;
}

void zeroEncoders() {
    encoderZeroValue1 = sensor.getEncoderValue(0);
    encoderZeroValue2 = sensor.getEncoderValue(4);
}

// Converts a raw encoder reading into "logical clicks" — dividing out
// ENCODER_COUNTS_PER_CLICK in case the hardware reports more than one
// raw count per physical detent (see the note near its #define).
int adjustedEncoderValue1(int rawValue) {
    return (rawValue - encoderZeroValue1) / ENCODER_COUNTS_PER_CLICK;
}

int adjustedEncoderValue2(int rawValue) {
    return (rawValue - encoderZeroValue2) / ENCODER_COUNTS_PER_CLICK;
}
