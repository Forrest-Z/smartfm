<?php
require("funcs.php");

$con = connect_to_DB();

$customerID = $_REQUEST["CustomerID"];
$vehicleID = $_REQUEST["VehicleID"];
$requestID = $_REQUEST["RequestID"];

$setvar = array();
if( isset($customerID) )
    $setvar[] = "CustomerID='$customerID'";
if( isset($vehicleID) )
    $setvar[] = "VehicleID='$vehicleID'";
if( isset($requestID) )
    $setvar[] = "RequestID=$requestID";

$query = "SELECT * FROM requests";
if( sizeof($setvar) )
    $query .= " WHERE " . join(' and ', $setvar);
$query .= " ORDER BY RequestID";

$xmlres = new XMLRes();

$result = mysql_query($query, $con) or $xmlres->fatalSqlError($query);

$parnode = $xmlres->addNode( $xmlres->createElement("requestList") );

while ($row = @mysql_fetch_assoc($result))
{
    $newnode = $parnode->appendChild( $xmlres->createElement("request") );
    $newnode->setAttribute("requestID", $row['requestID']);
    $newnode->setAttribute("customerID", $row['customerID']);
    $newnode->setAttribute("status", $row['status']);
    $newnode->setAttribute("pickup", $row['pickUpLocation']);
    $newnode->setAttribute("dropoff", $row['dropOffLocation']);
    $newnode->setAttribute("vehicleID", $row['vehicleID']);
    $newnode->setAttribute("eta", $row['eta']);
    $newnode->setAttribute("custCancelled", $row['custCancelled']);
    $newnode->setAttribute("vehicleAcknowledgedCancel", $row['vehicleAcknowledgedCancel']);

    $vstatus = "";
    if( isset($row['vehicleID']) and $row['vehicleID']!="" ) {
        $query = "SELECT status FROM vehicles WHERE vehicleID='" . $row['vehicleID'] . "'";
        $vresult = mysql_query($query, $con) or $xmlres->fatalSqlError($query);
        $vrow = @mysql_fetch_assoc($vresult);
        $vstatus = $vrow['status'];
    }
    $newnode->setAttribute("vehicleStatus", $vstatus);
}

mysql_close($con);
$xmlres->success();
?>
