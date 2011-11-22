<?php
require("funcs.php");

$con = connect_to_DB();

$vehicleID = $_REQUEST["VehicleID"];
if( isset($vehicleID) )
    $sql = "SELECT * FROM vehicles WHERE VehicleID = '$vehicleID'";
else
    $sql = "SELECT * FROM vehicles";
$result = mysql_query($sql, $con) or fatal('Select error: ' . mysql_error());

// Start XML file, create parent node
$doc = new DOMDocument("1.0");
$node = $doc->createElement("vehicleList");
$parnode = $doc->appendChild($node);

// Iterate through the rows, adding XML nodes for each
while ($row = @mysql_fetch_assoc($result)) {
    // ADD TO XML DOCUMENT NODE
    $node = $doc->createElement("vehicle");
    $newnode = $parnode->appendChild($node);
    $newnode->setAttribute("vehicleID", $row['vehicleID']);
    $newnode->setAttribute("status", $row['status']);
    $newnode->setAttribute("latitude", $row['latitude']);
    $newnode->setAttribute("longitude", $row['longitude']);
    $newnode->setAttribute("requestID", $row['requestID']);
    $newnode->setAttribute("eta", $row['eta']);
}

header("Content-type: text/xml");
echo $doc->saveXML();

mysql_close($con);
?>
