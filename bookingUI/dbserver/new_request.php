<?php
require("funcs.php");

$customerID = $_REQUEST["CustomerID"] or fatal('CustomerID missing');
$pickup = $_REQUEST["PickUpLocation"] or fatal('PickUpLocation missing');
$dropoff = $_REQUEST["DropOffLocation"] or fatal('DropOffLocation missing');

$con = connect_to_DB();

station_exists($con, $pickup) or fatal('PickUpLocation does not exist');
station_exists($con, $dropoff)) or fatal('DropOffLocation does not exist');

$sql = "INSERT INTO requests (CustomerID, Status, PickUpLocation, DropOffLocation) VALUES ('$customerID', 'Requested', '$pickup', '$dropoff')";
mysql_query($sql, $con) or fatal('Insert error: ' . mysql_error());
$requestID = mysql_insert_id();

mysql_close($con);
success("RequestID=$requestID");
?>
