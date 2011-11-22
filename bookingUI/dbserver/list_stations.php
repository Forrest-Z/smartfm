<?php
require("funcs.php");

$con = connect_to_DB();
$query = "SELECT * FROM stations";
$result = mysql_query($query) or fatal('Select error: ' . mysql_error());

// Start XML file, create parent node
$doc = new DOMDocument("1.0");
$node = $doc->createElement("stationList");
$parnode = $doc->appendChild($node);

// Iterate through the rows, adding XML nodes for each
while ($row = @mysql_fetch_assoc($result)){
    // ADD TO XML DOCUMENT NODE
    $node = $doc->createElement("station");
    $newnode = $parnode->appendChild($node);
    $newnode->setAttribute("name", $row['name']);
    $newnode->setAttribute("latitude", $row['latitude']);
    $newnode->setAttribute("longitude", $row['longitude']);
}

header("Content-type: text/xml");
echo $doc->saveXML();

mysql_close($con);
?>
