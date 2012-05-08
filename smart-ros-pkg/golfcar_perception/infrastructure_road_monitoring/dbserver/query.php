<?php
require("funcs.php");
$xmlres = new XMLRes();
$con = connect_to_DB();

$query = "select * from infrastructure";

if( isset($_REQUEST["Id"]) )
    $query .= " where id='" . $_REQUEST["Id"] . "'";

$result = mysql_query($query, $con) or $xmlres->fatalSqlError($query);
while ($row = @mysql_fetch_assoc($result))
{
    $newnode = $xmlres->addNode( $xmlres->createElement("infrastructure") );
    $newnode->setAttribute("id", $row['id']);
    $newnode->setAttribute("status", $row['status']);
}
mysql_close($con);
$xmlres->success();
?>
