<?php
require("funcs.php");
$con = connect_to_DB() or die('Connect error: ' . mysql_error());
$requestID = htmlspecialchars($_REQUEST["RequestID"]);
$sql = "UPDATE requests SET status = 'Cancelled' WHERE RequestID = '$requestID'";
$result = mysql_query($sql, $con) or die('Update error: ' . mysql_error());
mysql_close($con);
header( 'Location: administration.php' ) ;
?>