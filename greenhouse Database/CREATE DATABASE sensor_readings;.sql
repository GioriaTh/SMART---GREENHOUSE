CREATE DATABASE sensor_readings;


CREATE TABLE sensor_readings (
  id INT NOT NULL AUTO_INCREMENT,
  TIMESTAMP datetime,
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
  ir_in SMALLINT(5) UNSIGNED,
  ir_out SMALLINT(5) UNSIGNED,
  door INT(11),
  wind FLOAT,
  PRIMARY KEY (id)
);DROP TABLE sensor_readings if EXISTS;

