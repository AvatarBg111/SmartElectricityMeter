<?php
    header('Cache-Control: no cache'); //no cache
    session_cache_limiter('private_no_expire');
    session_start();

    $servername = "localhost";
    $dbname = "id15617968_tryinghard";
    $username = "id15617968_avatarbg555";
    $password = "3s?BC&=FY0Ujj>Bn";
    $consumer = $_POST["consumer"];
    $date = $_POST["date"];
    $consumer_even_consumed = $no_info_for_given_date = 0;
    $_SESSION["powers"] = array();

    $conn = new mysqli($servername, $username, $password, $dbname);    //Create connection
    $sql = "SHOW TABLES FROM ".$dbname;    //MySQL request string
    $tables = $conn->query($sql);    //Request and result transaction

    while($table = mysqli_fetch_assoc($tables)){
        $table_name = implode("", $table);
        $consumer_id = $_SESSION["consumers_ids"][array_search($consumer, $_SESSION["consumers_names"])];

        if(explode("_", $table_name)[0] != "consumers" && explode("_", $table_name)[1] == $consumer_id){
            $sql = "SELECT * FROM ".implode("", $table);
            $tab_info = $conn->query($sql);
            $info = mysqli_fetch_assoc($tab_info);
            if(explode(" ", $info["start_time"])[0] == $date){
                //echo "<br>Table name: ".implode("", $table)." Timestamp: ";
                $tab_info = $conn->query($sql);
                $last_hour = $last_mins = $last_power = $hour = $mins = $power = $flag = $flag_repeat = 0;   //$last_flag
                //WORK WITH INFO//
                for($i = 0; $i < 24; $i++)    array_push($_SESSION["powers"], 0);

                for($i = 0; $i < 24; $i++){
                    if($i == $hour){
                        $last_hour = (int)$hour;
                        $last_mins = (int)$mins;
                        if($flag == 1)  $last_power = $power;
                        //$last_flag = $flag;

                        $info = mysqli_fetch_assoc($tab_info);
                        if($info == NULL){
                            $hour = $i + 1;
                            array_push($_SESSION["powers"], 0);
                            //echo "<br>Index[$i]: Skipping to ".($i+1);
                            continue;
                        }//else    echo "<br>Sub-table information: ".implode("][", $info)." ARRAY INFO: ".implode(", ", $_SESSION["powers"]);
                        $hour = (int)explode(":", explode(" ", $info["start_time"])[1])[0];
                        $mins = (int)explode(":", explode(" ", $info["start_time"])[1])[1];
                        $power = $info["power"];
                        $flag = $info["flag"];  //RENAME IT TO STATE AFTER ALGORITHM IS DONE
                    }
                    if($flag == 0){
                        if($last_hour == $hour)    $_SESSION["powers"][$last_hour] += (int)(((float)(($mins - $last_mins) / 60)) * $last_power);
                        else if($last_hour == $hour - 1){
                            $_SESSION["powers"][$last_hour] = (int)(((float)((60 - $last_mins) / 60)) * $last_power);
                            $_SESSION["powers"][$hour] = (int)(((float)($mins/60)) * $last_power);
                            continue;
                        }else if($i == $last_hour)    $_SESSION["powers"][$i] += (int)(((float)((60 - $last_mins) / 60)) * $last_power);
                        else    $_SESSION["powers"][$i] = $last_power;
                        if($i == $hour - 1)    $_SESSION["powers"][$hour] = (int)(((float)($mins/60)) * $last_power);
                    }else if($flag == 1){
                        if($i == $hour)  $i--;
                    }else if($flag == 2){/*TO BE CONTINUED*/}
                }
                //WORK WITH INFO//
                $no_info_for_given_date = 1;
                break;
            }
            $no_info_for_given_date = 0;
            $consumer_even_consumed = 1;
        }
    }
    echo "<div id='consumer_info' style='border:1px solid #000000;'></div>";
    echo "<script>document.getElementById('consumer_info').innerHTML += 'The chosen consumer was: $consumer<br>The chosen date: $date';</script>";

    /*echo "<br>Consumption list: ".implode(" ", $_SESSION["powers"]);*/

    $conn->close();
    $graph_file = fopen("daily_consumption.csv", "w");
    fwrite($graph_file, "hour,powers\n");
    for($i = 0; $i < 24; $i++){
        if($consumer_even_consumed && $no_info_for_given_date)    fwrite($graph_file, "$i:00,".$_SESSION["powers"][$i]."\n");
        else    fwrite($graph_file, "$i:00,0\n");
    }
    if($consumer_even_consumed == 0)    echo "<script>document.getElementById('consumer_info').innerHTML += '<br>No consumer consumption registered!';</script>";
    else if($no_info_for_given_date == 0){
        echo "<script>document.getElementById('consumer_info').innerHTML += '<br>Consumer hasn\'t been consuming electricity on that date!';</script>";
    }else    echo "<script>document.getElementById('consumer_info').innerHTML += '<br>Consumer consumed electricity on that date!';</script>";

    fwrite($graph_file, "24:00,0\n");
    echo fread(fopen("graph.html", "r"), filesize("graph.html"));
?>