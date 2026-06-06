#include "datalog.h"
#include "config.h"
#include "globals.h"
#include <LittleFS.h>

uint32_t logRowCount = 0;

static const char CSV_HEADER[] =
    "ts_ms,pm1.0,pm2.5,pm4.0,pm10,rh_%,temp_C,voc_idx,nox_idx,co2_ppm\n";

void initLog() {
    if (!LittleFS.exists("/log.csv")) {
        File f = LittleFS.open("/log.csv", "w");
        if (f) { f.print(CSV_HEADER); f.close(); }
        logRowCount = 0;
        return;
    }
    File f = LittleFS.open("/log.csv", "r");
    logRowCount = 0;
    while (f.available()) {
        if (f.read() == '\n') logRowCount++;
    }
    f.close();
    if (logRowCount > 0) logRowCount--;  // subtract header
    Serial.printf("[log] Existing log: %lu rows\n", logRowCount);
}

void appendLog(const Measurement& m) {
    if (!m.valid) return;

    // Ring-buffer wrap
    if (logRowCount >= cfg.log_max) {
        Serial.println("[log] Ring wrap — trimming oldest entries");
        File src = LittleFS.open("/log.csv", "r");
        File tmp = LittleFS.open("/log.tmp", "w");
        if (!src || !tmp) return;

        uint32_t skipRows = logRowCount - (cfg.log_max - 1) + 1;
        uint32_t skipped  = 0;
        tmp.print(CSV_HEADER);
        while (src.available()) {
            String line = src.readStringUntil('\n');
            if (skipped < skipRows) { skipped++; continue; }
            if (line.length() > 5)  { tmp.print(line); tmp.print('\n'); }
        }
        src.close(); tmp.close();
        LittleFS.remove("/log.csv");
        LittleFS.rename("/log.tmp", "/log.csv");
        logRowCount = cfg.log_max - 1;
    }

    File f = LittleFS.open("/log.csv", "a");
    if (!f) return;

    char s1[10], s2[10], s3[10], s4[10];
    if (m.pm_sat) {
        strcpy(s1,"sat"); strcpy(s2,"sat");
        strcpy(s3,"sat"); strcpy(s4,"sat");
    } else {
        dtostrf(m.pm1,  1, 1, s1); dtostrf(m.pm25, 1, 1, s2);
        dtostrf(m.pm4,  1, 1, s3); dtostrf(m.pm10, 1, 1, s4);
    }
    f.printf("%lu,%s,%s,%s,%s,%.1f,%.2f,%.1f,%.1f,%u\n",
        m.ts_ms, s1, s2, s3, s4,
        m.rh, m.temp, m.voc, m.nox, m.co2);
    f.close();
    logRowCount++;
}

// ── History JSON ──────────────────────────────────────────────────────────
// Returns last `n` rows as JSON for the graph endpoint.
// Channel bitmask: bit0=temp, bit1=rh, bit2=co2,
//                  bit3=voc,  bit4=nox, bit5=pm25
String getHistoryJson(uint16_t n, uint8_t channels) {
    if (!LittleFS.exists("/log.csv")) return "[]";

    File f = LittleFS.open("/log.csv", "r");
    if (!f) return "[]";

    // Collect last n lines into a circular string buffer
    // We do a two-pass: first count lines, then re-read from (total-n)
    uint32_t total = 0;
    while (f.available()) { if (f.read() == '\n') total++; }
    if (total > 0) total--; // subtract header

    uint32_t skip = (total > n) ? total - n : 0;

    f.seek(0);
    // Skip header
    f.readStringUntil('\n');
    // Skip oldest rows
    for (uint32_t i = 0; i < skip; i++) f.readStringUntil('\n');

    // Build JSON
    // Format: {"labels":[...],"temp":[...],"rh":[...],...}
    String out = "{";

    // Parse all target lines into arrays
    // Columns: ts_ms,pm1.0,pm2.5,pm4.0,pm10,rh_%,temp_C,voc_idx,nox_idx,co2_ppm
    //   idx:    0     1     2     3     4    5     6      7       8       9

    struct Series {
        const char* key;
        uint8_t     col;   // CSV column index
        uint8_t     bit;   // channel bitmask bit
    };
    static const Series series[] = {
        {"temp",  6, 0},
        {"rh",    5, 1},
        {"co2",   9, 2},
        {"voc",   7, 3},
        {"nox",   8, 4},
        {"pm25",  2, 5},
    };

    // We'll build each array in one pass
    // Store lines temporarily
    const uint16_t MAX_LINES = 120;
    static String  lines[MAX_LINES];
    uint16_t       lineCount = 0;

    while (f.available() && lineCount < MAX_LINES) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() > 5) lines[lineCount++] = line;
    }
    f.close();

    // Labels (relative seconds from first reading)
    out += "\"labels\":[";
    uint32_t t0 = 0;
    for (uint16_t i = 0; i < lineCount; i++) {
        // Extract ts_ms (col 0)
        int comma = lines[i].indexOf(',');
        uint32_t ts = (comma > 0) ? lines[i].substring(0, comma).toInt() : 0;
        if (i == 0) t0 = ts;
        if (i > 0) out += ',';
        out += String((ts - t0) / 1000);  // seconds since first point
    }
    out += "],";

    // Each series
    for (uint8_t s = 0; s < 6; s++) {
        if (!(channels & (1 << series[s].bit))) continue;

        out += "\"";
        out += series[s].key;
        out += "\":[";

        for (uint16_t i = 0; i < lineCount; i++) {
            // Parse CSV column
            const String& line = lines[i];
            uint8_t col = 0;
            int start = 0, end = 0;
            while (col < series[s].col) {
                start = line.indexOf(',', start) + 1;
                col++;
            }
            end = line.indexOf(',', start);
            String val = (end < 0)
                ? line.substring(start)
                : line.substring(start, end);
            val.trim();

            if (i > 0) out += ',';
            if (val == "sat" || val == "null" || val.length() == 0)
                out += "null";
            else
                out += val;
        }
        out += "],";
    }

    // Remove trailing comma before closing
    if (out.endsWith(",")) out.remove(out.length() - 1);
    out += "}";
    return out;
}
