<?php
    header('Cache-Control: no cache'); //no cache
    session_cache_limiter('private_no_expire');
    session_start();

    //Database host
    $servername = "localhost";
    //Database name
    $dbname = "id15617968_tryinghard";
    //Database user
    $username = "id15617968_avatarbg555";
    //Database user password
    $password = "3s?BC&=FY0Ujj>Bn";

    // Create connection
    $conn = new mysqli($servername, $username, $password, $dbname);
    // Check connection
    if ($conn->connect_error)  die("Connection failed: " . $conn->connect_error);

    $consumers_names = array();
    $consumers_ids = array();
    $_SESSION["consumers_names"] = array();
    $_SESSION["consumers_ids"] = array();

    $sql = "SELECT * FROM consumers";
    $info = $conn->query($sql);
    if(!$info){
        echo "DB Error, could not list tables\n";
        exit;
    }else{
        while($res = mysqli_fetch_assoc($info)){
            array_push($consumers_names, $res["consumer_name"]);
            array_push($consumers_ids, $res["consumer_id"]);

            array_push($_SESSION["consumers_names"], $res["consumer_name"]);
            array_push($_SESSION["consumers_ids"], $res["consumer_id"]);
        }

        $conn->close();
        echo fread(fopen("show_graph.html", "r"), filesize("show_graph.html"));
        for($i = 0; $i < count($consumers_names); $i++){
            echo "<script>
                     var option = document.createElement('option');
                     option.value = '$consumers_names[$i]';
                     option.innerHTML = '$consumers_names[$i]';
                     document.getElementById('consumer').appendChild(option);
                 </script>";
        }
    }
?>