<?php
session_start();

// Function to load variable from environment variables, if not set, use default value
function env($key, $default = null)
{
    $value = getenv($key);
    if ($value === false) {
        return $default;
    }
    return $value;
}

// Database configuration load fron env variables, if not set, use default values
$host = env('DB_HOST', '127.0.0.1');
$user = env('DB_USERNAME', 'robotnik');
$password = env('DB_PASS', 'R0b0tn1K');
$dbname = env('DB_DATABASE', 'db_robotnik');

// Create connection
$con = mysqli_connect($host, $user, $password, $dbname);
// Check connection
if (!$con) {
 die("Connection failed: " . mysqli_connect_error());
}
