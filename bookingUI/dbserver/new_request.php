<?php
require("funcs.php");

$xmlres = new XMLRes();
$con = connect_to_DB();

$customerID = $_REQUEST["CustomerID"] or $xmlres->fatal('CustomerID missing');
$pickup = $_REQUEST["PickUpLocation"] or $xmlres->fatal('PickUpLocation missing');
$dropoff = $_REQUEST["DropOffLocation"] or $xmlres->fatal('DropOffLocation missing');

station_exists($con, $pickup) or $xmlres->fatal("PickUpLocation $pickup does not exist");
station_exists($con, $dropoff) or $xmlres->fatal("DropOffLocation $dropoff does not exist");

$query = "INSERT INTO requests (CustomerID, Status, PickUpLocation, DropOffLocation) VALUES ('$customerID', 'Requested', '$pickup', '$dropoff')";
mysql_query($query, $con) or $xmlres->fatalSqlError($query);
$requestID = mysql_insert_id();

mysql_close($con);

$newnode = $xmlres->addNode( $xmlres->createElement("taskadded") );
$newnode->setAttribute("id", $requestID);

$xmlres->success();
?>
