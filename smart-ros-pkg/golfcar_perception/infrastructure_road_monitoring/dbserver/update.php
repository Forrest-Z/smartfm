<?php
require("funcs.php");
$xmlres = new XMLRes();
$con = connect_to_DB();

if( isset($_REQUEST["Id"]) )
    $id = $_REQUEST["Id"];
else
    $xmlres->fatal('Id missing');

if( isset($_REQUEST["Status"]) )
    $status = $_REQUEST["Status"];
else
    $xmlres->fatal('Status missing');

$query = "INSERT INTO infrastructure (Id, Status) VALUES ('$id', $status) ON DUPLICATE KEY UPDATE Status=$status";
mysql_query($query, $con) or $xmlres->fatalSqlError($query);
mysql_close($con);
$xmlres->success();
?>
