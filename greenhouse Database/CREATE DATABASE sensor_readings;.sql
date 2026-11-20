-- Schema for the Smart Greenhouse sensor data.
-- The table name must match the INSERT in db_sensor_data.php.

CREATE DATABASE IF NOT EXISTS sensor_readings;
USE sensor_readings;

DROP TABLE IF EXISTS sens_readings;

CREATE TABLE sens_readings (
  id INT NOT NULL AUTO_INCREMENT,
  timestamp DATETIME,
  temp1 FLOAT,
  temp2 FLOAT,
  temp3 FLOAT,
  temp4 FLOAT,
  temp5 FLOAT,
  temp6 FLOAT,
  temp7 FLOAT,
  temp8 FLOAT,
  temp9 FLOAT,
  temp10 FLOAT,
  temp11 FLOAT,
  temp12 FLOAT,
  chtIn_humidity FLOAT,
  chtIn_temp FLOAT,
  chtOut_humidity FLOAT,
  chtOut_temp FLOAT,
  ir_in SMALLINT UNSIGNED,
  ir_out SMALLINT UNSIGNED,
  door INT,
  wind FLOAT,
  PRIMARY KEY (id)
);
