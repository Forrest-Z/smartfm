<?php
require("funcs.php");

$con = connect_to_DB();

$customerID = $_REQUEST["CustomerID"];
if( isset($customerID) )
    $sql = "SELECT * FROM requests WHERE CustomerID = '$customerID'";
else
    $sql = "SELECT * FROM requests";
$result = mysql_query($sql, $con) or fatal('Select error: ' . mysql_error());

// Start XML file, create parent node
$doc = new DOMDocument("1.0");
$node = $doc->createElement("requestList");
$parnode = $doc->appendChild($node);

// Iterate through the rows, adding XML nodes for each
while ($row = @mysql_fetch_assoc($result)) {
    // ADD TO XML DOCUMENT NODE
    $node = $doc->createElement("request");
    $newnode = $parnode->appendChild($node);
    $newnode->setAttribute("requestID", $row['requestID']);
    $newnode->setAttribute("customerID", $row['customerID']);
    $newnode->setAttribute("status", $row['status']);
    $newnode->setAttribute("pickup", $row['pickUpLocation']);
    $newnode->setAttribute("dropoff", $row['dropOffLocation']);
    if( $row['status']=="Confirmed" || $row['status']=="Processing" ) {
        $vid = $row['vehicleID'];
        $newnode->setAttribute("vehicleID", $vid);
        $sql = "SELECT * FROM vehicles WHERE VehicleID = '$vid'";
        $res = mysql_query($sql, $con) or fatal('Select error: ' . mysql_error());
        while ($r = @mysql_fetch_assoc($res)) {
            $newnode->setAttribute("latitude", $r['latitude']);
            $newnode->setAttribute("longitude", $r['longitude']);
            $newnode->setAttribute("eta", $r['eta']);
        }
    }
}

header("Content-type: text/xml");
echo $doc->saveXML();

mysql_close($con);
?>
