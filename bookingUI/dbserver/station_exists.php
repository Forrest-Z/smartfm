<?php
require("funcs.php");
$xmlres = new XMLRes();
$con = connect_to_DB();

$name = $_REQUEST["Name"] or $xmlres->fatal('Name missing.');

$res = "Station $name does not exist.";
if (station_exists($con, $name))
    $res = "Station $name exists.";

mysql_close($con);
$xmlres->success($res);
?>
