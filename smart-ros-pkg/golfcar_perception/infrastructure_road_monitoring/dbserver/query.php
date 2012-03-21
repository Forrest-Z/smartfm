<?php
require("funcs.php");
$xmlres = new XMLRes();
$con = connect_to_DB();

$query = "select go from tjunction";
$result = mysql_query($query, $con) or $xmlres->fatalSqlError($query);
while ($row = @mysql_fetch_assoc($result))
{
    $newnode = $xmlres->addNode( $xmlres->createElement("go") );
    $newnode->setAttribute("status", $row['go']);
}
mysql_close($con);
$xmlres->success();
?>
