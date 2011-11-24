<?php
require("credentials.php");

//ini_set('display_errors', 'On');
//error_reporting(E_ALL | E_STRICT);

class XMLRes
{
    public $doc;
    public $root;

    public function __construct()
    {
        header("Content-type: text/xml");
        $this->doc = new DOMDocument("1.0");
        $this->doc->formatOutput = true;
        $this->root = $this->doc->appendChild( $this->doc->createElement("rootxml") );
    }

    public function createElement($name)
    {
        return $this->doc->createElement($name);
    }

    public function addNode($node)
    {
        return $this->root->appendChild($node);
    }


    private function appendStatusNode($status, $msg="")
    {
        $node = $this->addNode( $this->createElement("status") );
        if( $status )
            $node->setAttribute("code", "ok");
        else
            $node->setAttribute("code", "err");
        $node->setAttribute("msg", $msg);
        return $node;
    }

    public function fatal($msg="")
    {
        $this->appendStatusNode(0,$msg);
        echo $this->doc->saveXML();
        exit(1);
    }

    public function success($msg="")
    {
        $this->appendStatusNode(1,$msg);
        echo $this->doc->saveXML();
        exit(0);
    }
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
