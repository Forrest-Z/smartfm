<?php
require("funcs.php");

$name = $_REQUEST["Name"] or fatal('Name missing.');

$con = connect_to_DB();

if (station_exists($con, $name))
    success("Station $name exists.");
else
    success("Station $name does not exist.");

mysql_close($con);
?>
