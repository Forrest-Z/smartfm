<?php
require("funcs.php");

$requestID = $_REQUEST["RequestID"] or fatal('RequestID missing');
$status = $_REQUEST["Status"] or fatal('Status missing');

$con = connect_to_DB();
$sql = "UPDATE requests SET status = '$status' WHERE RequestID = '$requestID'";
mysql_query($sql, $con) or fatal('Update error: ' . mysql_error());

$n = mysql_affected_rows();
mysql_close($con);
success("Cancelled $n rows");
?>