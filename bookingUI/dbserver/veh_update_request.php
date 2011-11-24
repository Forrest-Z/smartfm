<?php
require("funcs.php");
$xmlres = new XMLRes();
$requestID = $_REQUEST["RequestID"] or $xmlres->fatal('RequestID missing');
$status = $_REQUEST["Status"] or $xmlres->fatal('Status missing');

$con = connect_to_DB();
$sql = "UPDATE requests SET status = '$status' WHERE RequestID = '$requestID'";
mysql_query($sql, $con) or $xmlres->fatal('Update error: ' . mysql_error());

$n = mysql_affected_rows();
mysql_close($con);
$xmlres->success("Updated $n rows");
?>