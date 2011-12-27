<?php
require("funcs.php");

$con = connect_to_DB();
$xmlres = new XMLRes();

$customerID = $_REQUEST["CustomerID"] or $xmlres->fatal('CustomerID missing');
$requestID = $_REQUEST["RequestID"];

$query = "UPDATE requests SET custCancelled=1 WHERE CustomerID='$customerID'";
if( isset($requestID) )
    $query .= " and RequestID=$requestID";

$result = mysql_query($query, $con) or $xmlres->fatalSqlError($query);

$n = mysql_affected_rows();

mysql_close($con);
$xmlres->success("Cancelled $n rows");
?>
