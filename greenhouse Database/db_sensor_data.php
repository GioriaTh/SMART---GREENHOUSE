<?php
$servername = "";
$username = "";
$password = "";
$dbname = "sensor_readings";

// Connect to MySQL
$conn = new mysqli($servername, $username, $password, $dbname);
if ($conn->connect_error) {
    die("Connection failed: " . $conn->connect_error);
}

// Read JSON data from ESP32
$jsonData = file_get_contents("php://input");
$data = json_decode($jsonData, true);

// FIX: Ensure JSON is an **array** (Handles single-object case)
if (!$data) {
    die("Invalid JSON data! Received: " . $jsonData);
}
if (!is_array(reset($data))) {
    $data = [$data]; // Convert single object into an array
}

// Debug JSON data
file_put_contents("debug_log.txt", "Received JSON:\n" . print_r($data, true) . "\n", FILE_APPEND);

// Prepare bulk insert
$sql = "INSERT INTO sens_readings (timestamp, temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp9, temp10, temp11, temp12, chtIn_humidity, chtIn_temp, chtOut_humidity, chtOut_temp, ir_in,ir_out, door, wind) VALUES ";

$values = [];

foreach ($data as $row) {
    $values[] = "('" . $conn->real_escape_string($row["timestamp"] ?? date("Y-m-d H:i:s")) . "', " .
                floatval($row["temp1"] ?? 0) . ", " . floatval($row["temp2"] ?? 0) . ", " . floatval($row["temp3"] ?? 0) . ", " . floatval($row["temp4"] ?? 0) . ", " . 
                floatval($row["temp5"] ?? 0) . ", " . floatval($row["temp6"] ?? 0) . ", " . floatval($row["temp7"] ?? 0) . ", " . floatval($row["temp8"] ?? 0) . ", " . 
                floatval($row["temp9"] ?? 0) . ", " . floatval($row["temp10"] ?? 0) . ", " . floatval($row["temp11"] ?? 0) . ", " . floatval($row["temp12"] ?? 0) . ", " . 
                floatval($row["chtIn_humidity"] ?? 0) . ", " . floatval($row["chtIn_temp"] ?? 0) . ", " . floatval($row["chtOut_humidity"] ?? 0) . ", " . 
                floatval($row["chtOut_temp"] ?? 0) . ", " . intval($row["ir_in"] ?? 0) . ", " . intval($row["ir_out"] ?? 0) . ", " . intval($row["door"] ?? 0) . ", " . floatval($row["wind"] ?? 0) . ")";
}

$sql .= implode(",", $values);

// Debug SQL query
file_put_contents("debug_log.txt", "Generated SQL Query:\n" . $sql . "\n", FILE_APPEND);

// Execute the SQL query
if ($conn->query($sql) === TRUE) {
    echo "Data Inserted Successfully!";
} else {
    echo "SQL Error: " . $conn->error;
    file_put_contents("debug_log.txt", "SQL Error: " . $conn->error . "\n", FILE_APPEND);
}

// Close connection
$conn->close();
?>
