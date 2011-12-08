<?php
require("funcs.php");

$requestID = htmlspecialchars($_REQUEST["RequestID"]) or die('RequestID missing');
$filter = htmlspecialchars($_REQUEST['filter']);

$con = connect_to_DB() or die('Connect error: ' . mysql_error());
$sql = "UPDATE requests SET status = 'Cancelled' WHERE RequestID = '$requestID'";
$result = mysql_query($sql, $con) or die('Update error: ' . mysql_error());
mysql_close($con);

if( !isset($filter) || $filter!='yes' )
$filter = 'no';
header( "Location: administration.php?filter=$filter" );
?>
