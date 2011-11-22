<?php
require("credentials.php");

//ini_set('display_errors', 'On');
//error_reporting(E_ALL | E_STRICT);

function fatal($msg=NULL)
{
    header("Content-type: text/xml");
    if( $msg )
        echo("<status code='err' msg='$msg'/>");
    else
        echo("<status code='err'/>");
    exit(1);
}

function success($msg=NULL)
{
    header("Content-type: text/xml");
    if( $msg )
        echo "<status code='ok' msg='$msg'/>";
    else
        echo "<status code='ok'/>";
    exit(0);
}

function connect_to_DB()
{
    global $username, $password, $database;
    $con = mysql_connect ('localhost', $username, $password) or fatal('Not connected : ' . mysql_error());
    mysql_select_db($database, $con) or fatal('Can\'t use db : ' . mysql_error());
    return $con;
}


function station_exists($con, $name)
{
    $query = "SELECT * FROM stations WHERE name='$name'";
    $result = mysql_query($query) or fatal('Select error: ' . mysql_error());
    while( $row = mysql_fetch_assoc($result) )
        return true;
    return false;
}

?>
