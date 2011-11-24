<?php
require("funcs.php");

$xmlres = new XMLRes();

$con = connect_to_DB();
$query = "SELECT * FROM stations";
$result = mysql_query($query) or $xmlres->fatal('Select error: ' . mysql_error());

$parnode = $xmlres->addNode( $xmlres->createElement("stationList") );

// Iterate through the rows, adding XML nodes for each
while ($row = @mysql_fetch_assoc($result))
{
    $newnode = $parnode->appendChild( $xmlres->createElement("station") );
    $newnode->setAttribute("name", $row['name']);
    $newnode->setAttribute("latitude", $row['latitude']);
    $newnode->setAttribute("longitude", $row['longitude']);
}

mysql_close($con);
$xmlres->success();
?>
