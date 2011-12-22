<?php
require("funcs.php");
$xmlres = new XMLRes();
$con = connect_to_DB();

$requestID = $_REQUEST["RequestID"] or $xmlres->fatal('RequestID missing');

$query = "UPDATE requests SET vehicleAcknowledgedCancel='Rejected' WHERE RequestID =$requestID";
mysql_query($query, $con) or $xmlres->fatalSqlError($query);

$n = mysql_affected_rows();
mysql_close($con);
$xmlres->success("Updated $n rows");
?>
