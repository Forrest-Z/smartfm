<?php
require("funcs.php");
$xmlres = new XMLRes();
$con = connect_to_DB();

$status = $_REQUEST["Status"] or $xmlres->fatal('Status missing');
$query = "UPDATE tjunction SET go=$status";
mysql_query($query, $con) or $xmlres->fatalSqlError($query);
mysql_close($con);
$xmlres->success();
?>
