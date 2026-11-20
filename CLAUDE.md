# CLAUDE.md — SMART---GREENHOUSE

## Purpose

ESP32 firmware + PHP/MySQL backend (table `sens_readings` in database `sensor_readings`) that collect greenhouse microclimate data. The data feeds the
ML work in the sibling repo `Greenhouse-AI`.

## Structure

- `greenhouse IoT/Esp32 ArduinoIDE.ino` — firmware (Arduino IDE, ESP32 core)
- `greenhouse Database/db_sensor_data.php` — receives a JSON array via HTTP POST, bulk-inserts into MySQL
- `greenhouse Database/CREATE DATABASE sensor_readings;.sql` — schema

## Data contract

The JSON keys sent by the firmware, the PHP insert columns and the SQL table columns must match:
`timestamp, temp1..temp12, chtIn_humidity, chtIn_temp, chtOut_humidity, chtOut_temp, ir_in, ir_out, door, wind`.
When you change one of the three, update the other two and the README payload example.

## Conventions

- Never commit real Wi-Fi credentials, server IPs or DB passwords. Keep them empty in code
  or move them to a `secrets.h` / `config.php` that is in `.gitignore`.
- Pin numbers and sensor list are documented in the README "Hardware" table — keep it in sync.
- Keep the low-power design: buffer to SPIFFS, Wi-Fi on only during upload.
- Use prepared statements in PHP for any new queries.

## Possible improvements

- File names contain spaces and a `;` — consider `firmware/greenhouse.ino`, `database/schema.sql`,
  `database/db_sensor_data.php` (ask the owner before renaming).
- Move Wi-Fi credentials and the server URL into a git-ignored `secrets.h`.

## Do not

- Do not commit `debug_log.txt` (written by the PHP script).
- Do not rewrite git history.
