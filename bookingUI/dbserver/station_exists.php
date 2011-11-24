<?php
require("funcs.php");
$xmlres = new XMLRes();
$name = $_REQUEST["Name"] or $xmlres->fatal('Name missing.');

$con = connect_to_DB();

$res = "Station $name does not exist.";
if (station_exists($con, $name))
    $res = "Station $name exists.";

mysql_close($con);
$xmlres->success($res);
?>
