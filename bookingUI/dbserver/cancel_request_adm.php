<?php
require("funcs.php");

$con = connect_to_DB() or die('Connect error: ' . mysql_error());

$requestID = $_REQUEST["RequestID"] or die('RequestID missing');
$filter = $_REQUEST['filter'];
$returnScript = $_REQUEST["ReturnScript"];

$sql = "UPDATE requests SET custCancelled=1 WHERE RequestID=$requestID";
$result = mysql_query($sql, $con) or die('Update error: ' . mysql_error());
mysql_close($con);

if( !isset($filter) || $filter!='yes' ) $filter = 'no';
if( !isset($returnScript) ) $returnScript = 'administration';
header( "Location: $returnScript.php?filter=$filter" );
?>
