<?php
require("funcs.php");

$vehicleID = $_REQUEST["VehicleID"] or fatal('VehicleID missing');
$con = connect_to_DB();

$sql = "DELETE FROM vehicles WHERE VehicleID='$vehicleID'";
mysql_query($sql, $con) or fatal('Delete error: ' . mysql_error());

$sql = "INSERT INTO vehicles (VehicleID, Status) VALUES ('$vehicleID', 'WaitingForAMission')";
mysql_query($sql, $con) or fatal('Insert error: ' . mysql_error());

mysql_close($con);
success();
?>