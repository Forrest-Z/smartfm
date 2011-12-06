<?php
require("funcs.php");

$xmlres = new XMLRes();

$customerID = $_REQUEST["CustomerID"] or $xmlres->fatal('CustomerID missing');
$pickup = $_REQUEST["PickUpLocation"] or $xmlres->fatal('PickUpLocation missing');
$dropoff = $_REQUEST["DropOffLocation"] or $xmlres->fatal('DropOffLocation missing');

$con = connect_to_DB();

station_exists($con, $pickup) or $xmlres->fatal('PickUpLocation does not exist');
station_exists($con, $dropoff) or $xmlres->fatal('DropOffLocation does not exist');

$sql = "INSERT INTO requests (CustomerID, Status, PickUpLocation, DropOffLocation) VALUES ('$customerID', 'Requested', '$pickup', '$dropoff')";
mysql_query($sql, $con) or $xmlres->fatal('Insert error: ' . mysql_error());
$requestID = mysql_insert_id();

mysql_close($con);
$xmlres->success("RequestID=$requestID");
?>
