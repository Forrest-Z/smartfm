<?php
require("funcs.php");
$xmlres = new XMLRes();
$vehicleID = $_REQUEST["VehicleID"] or $xmlres->fatal('VehicleID missing');

$con = connect_to_DB();
$sql = "INSERT INTO vehicles (VehicleID, Status) VALUES ('$vehicleID', 'WaitingForAMission')";
mysql_query($sql, $con) or $xmlres->fatal('Insert error: ' . mysql_error());
mysql_close($con);
$xmlres->success();
?>
