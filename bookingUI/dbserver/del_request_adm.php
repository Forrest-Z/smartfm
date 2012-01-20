<?php
require("funcs.php");

$con = connect_to_DB() or die('Connect error: ' . mysql_error());

$requestID = $_REQUEST["RequestID"] or die('RequestID missing');
$filter = $_REQUEST['filter'];
$returnScript = $_REQUEST["ReturnScript"];

$sql = "DELETE FROM requests WHERE RequestID=$requestID";
$result = mysql_query($sql, $con) or die('Delete error: ' . mysql_error());
mysql_close($con);

if( !isset($filter) || $filter!='yes' ) $filter = 'no';
if( !isset($returnScript) ) $returnScript = 'administration.php';
header( "Location: $returnScript?filter=$filter" );
?>
