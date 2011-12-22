<?php
require("funcs.php");

$con = connect_to_DB();
$xmlres = new XMLRes();

$vehicleID = $_REQUEST["VehicleID"] or $xmlres->fatal('VehicleID missing');

$query = "DELETE FROM vehicles WHERE VehicleID='$vehicleID'";
mysql_query($query, $con) or $xmlres->fatalSqlError($query);

mysql_close($con);
$xmlres->success();
?>
