<?php
require("funcs.php");

$customerID = $_REQUEST["CustomerID"] or die('CustomerID missing');
$pickup = $_REQUEST["PickUpLocation"] or die('PickUpLocation missing');
$dropoff = $_REQUEST["DropOffLocation"] or die('DropOffLocation missing');
$status = $_REQUEST["Status"] or die('Status missing');
$vehicleID = $_REQUEST["VehicleID"] or die('VehicleID missing');

$con = connect_to_DB();

station_exists($con, $pickup) or die('PickUpLocation does not exist');
station_exists($con, $dropoff) or die('DropOffLocation does not exist');

$sql = "INSERT INTO requests (CustomerID, Status, PickUpLocation, DropOffLocation, VehicleID) VALUES ('$customerID', '$status', '$pickup', '$dropoff', '$vehicleID')";
mysql_query($sql, $con) or die('Insert error: ' . mysql_error());

mysql_close($con);

header( 'Location: administration.php' ) ;
?>