<?php
require("funcs.php");

$xmlres = new XMLRes();

$vehicleID = $_REQUEST["VehicleID"] or $xmlres->fatal('VehicleID missing');
$con = connect_to_DB();

$sql = "DELETE FROM vehicles WHERE VehicleID='$vehicleID'";
mysql_query($sql, $con) or $xmlres->fatal('Delete error: ' . mysql_error());

mysql_close($con);
$xmlres->success();
?>
