# Smart Greenhouse — ESP32 IoT Monitoring

![ESP32](https://img.shields.io/badge/ESP32-Arduino-E7352C?logo=espressif&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-firmware-00599C?logo=cplusplus&logoColor=white)
![PHP](https://img.shields.io/badge/PHP-API-777BB4?logo=php&logoColor=white)
![MySQL](https://img.shields.io/badge/MySQL-database-4479A1?logo=mysql&logoColor=white)
![License](https://img.shields.io/badge/license-MIT-green)

A low-power IoT system that monitors the microclimate of a greenhouse. An ESP32 reads 18 sensors,
buffers the readings in flash memory, and periodically uploads them in batches to a PHP endpoint
that stores them in MySQL. The collected data is analysed in
[Greenhouse-AI](https://github.com/GioriaTh/Greenhouse-AI).

## Architecture

```
 Sensors ──► ESP32 ──(buffer in SPIFFS)──► Wi-Fi on ──HTTP POST JSON──► db_sensor_data.php ──► MySQL
                 ▲                              │
                 └──── Wi-Fi off between uploads┘
```

- Time is synchronised over NTP (`gr.pool.ntp.org`) and set on the ESP32 RTC.
- Readings are appended to `/ntc_data.txt` on SPIFFS, then uploaded in one batch and cleared.
- Wi-Fi stays **off** between uploads to save power.

## Hardware

| Sensor | Qty | Measures | Interface / pins |
|---|---|---|---|
| NTC thermistor 10 kΩ (β = 3950) | 12 | Temperature at 12 points | ADC: 39, 36, 35, 34, 32, 33, 25, 26, 27, 4, 13, 15 |
| CHT8305 | 2 | Temperature + humidity (inside / outside) | I²C bus 1 (SDA 21, SCL 22), bus 2 (SDA 16, SCL 17) |
| Adafruit TSL2591 | 2 | Light (inside / outside) | I²C bus 1 and bus 2 |
| Magnetic reed switch | 1 | Door open / closed | GPIO 5 |
| Analog anemometer (0–5 V → 0–30 m/s) | 1 | Wind speed | GPIO 14 |

## Repository structure

```
SMART---GREENHOUSE/
├── greenhouse IoT/
│   └── Esp32 ArduinoIDE.ino                   # ESP32 firmware
└── greenhouse Database/
    ├── CREATE DATABASE sensor_readings;.sql   # database schema
    └── db_sensor_data.php                     # HTTP endpoint: JSON → MySQL
```

## Setup

### 1. Database and server

1. Install a web server with PHP and MySQL (e.g. XAMPP / LAMP).
2. Make sure MySQL is running.
3. Run the SQL file, e.g. `mysql -u root -p < "greenhouse Database/CREATE DATABASE sensor_readings;.sql"`.
4. Copy `db_sensor_data.php` into the web root and fill in `$servername`, `$username`, `$password`.

### 2. Firmware

1. Open `greenhouse IoT/Esp32 ArduinoIDE.ino` in the Arduino IDE with the ESP32 board package.
2. Install the libraries: `NTPClient`, `PubSubClient`, `Adafruit Unified Sensor`, `Adafruit TSL2591`,
   `CHT8305`, `ArduinoJson`.
3. Set `ssid` / `password` and change `http.begin("http://192.x.x.x/db_sensor_data.php")` to your server address.
4. Adjust `SAVE_INTERVAL` (sampling) and `UPLOAD_INTERVAL` (upload) if needed, then upload to the board.

### JSON payload

```json
[{ "timestamp": "2025-04-28 11:07:00", "temp1": 35.44, "...": "...", "temp12": 34.32,
   "chtIn_humidity": 29.7, "chtIn_temp": 34.8, "chtOut_humidity": 22.8, "chtOut_temp": 23.2,
   "ir_in": 6183, "ir_out": 6183, "door": 1, "wind": 0.5 }]
```

## Notes

- Default timing: one reading every 3 s (`SAVE_INTERVAL`), upload every 1 min (`UPLOAD_INTERVAL`).
- The SQL file drops and recreates the `sens_readings` table — don't run it on a database with data you want to keep.

## Related projects

- [Greenhouse-AI](https://github.com/GioriaTh/Greenhouse-AI) — ML analysis and LSTM virtual sensors on this data

## Author

**GioriaTh** — Computer Engineer · [github.com/GioriaTh](https://github.com/GioriaTh)

## License

Released under the [MIT License](LICENSE).
