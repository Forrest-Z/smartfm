<?php
require("funcs.php");
$xmlres = new XMLRes();
$con = connect_to_DB();

$id = $_REQUEST["Id"] or $xmlres->fatal('Id missing');
$status = $_REQUEST["Status"] or $xmlres->fatal('Status missing');

$query = "INSERT INTO infrastructure (Id, Status) VALUES ('$id', $status) ON DUPLICATE KEY UPDATE Status=$status";
mysql_query($query, $con) or $xmlres->fatalSqlError($query);
mysql_close($con);
$xmlres->success();
?>