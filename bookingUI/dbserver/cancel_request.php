<?php
require("funcs.php");

$con = connect_to_DB();

$xmlres = new XMLRes();

$customerID = $_REQUEST["CustomerID"] or $xmlres->fatal('CustomerID missing');

$requestID = $_REQUEST["RequestID"];
if( isset($requestID) )
    $sql = "UPDATE requests SET status = 'Cancelled' WHERE CustomerID = '$customerID' and RequestID = '$requestID'";
else
    $sql = "UPDATE requests SET status = 'Cancelled' WHERE CustomerID = '$customerID'";

$result = mysql_query($sql, $con) or $xmlres->fatal('Update error: ' . mysql_error());

$n = mysql_affected_rows();

mysql_close($con);
$xmlres->success("Cancelled $n rows");
?>