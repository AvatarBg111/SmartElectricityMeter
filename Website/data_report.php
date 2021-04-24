<?php
    header('Cache-Control: no cache'); //no cache
    session_cache_limiter('private_no_expire');
    session_start();

    $servername = "localhost";
    $dbname = "id15617968_tryinghard";
    $username = "id15617968_avatarbg555";
    $password = "3s?BC&=FY0Ujj>Bn";

    $conn = new mysqli($servername, $username, $password, $dbname);
    if ($conn->connect_error)  die("Connection failed: " . $conn->connect_error);

    $_SESSION["cons_names"] = array();
    $_SESSION["cons_ids"] = array();

    $sql = "SELECT * FROM consumers";
    $info = $conn->query($sql);
    if(!$info){
        echo "DB Error, could not list tables\n";
        exit;
    }else{
        while($res = mysqli_fetch_assoc($info)){
            array_push($_SESSION["cons_names"], $res["consumer_name"]);
            array_push($_SESSION["cons_ids"], $res["consumer_id"]);
        }

        $conn->close();
        echo fread(fopen("report_customization.html", "r"), filesize("report_customization.html"));
    }
?>