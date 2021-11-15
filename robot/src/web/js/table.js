function add_row(_name, _categ, _topic, _channel, _library, _status){
    // get table and rows object
    var table = document.getElementById("tbl");
 
    var rows = document.getElementById("tbl").length
    // create new row
    var row = table.insertRow(rows);

    // insert name cell
    var name = row.insertCell(0);

    // set html content for name cell
    name.innerHTML = "<p class=\"text-uppercase font-weight-bold\"><i class=\"fas fa-chevron-up\"></i> &nbsp" + _name  +"</p>"

    // create IDs based on the thread_name
    name_clean = _name.toLowerCase().replace(' ', '_') + "_name"
    name.id = name_clean

    // set the name_cell as a trigger for the hidden log row
    // aka this oppens the drawer for this row
    $("#" + name_clean).attr('data-target','#collapse_'  + name_clean);
    $("#" + name_clean).attr('data-toggle','collapse');

    var categ = row.insertCell(1);
    categ.innerHTML = "<p class=\"text-muted\">" + _categ +"</p>";

    var topic = row.insertCell(2);
    topic.innerHTML = "<p class=\"text-muted\">" + _topic +"</p>";

    var channel = row.insertCell(3);
    channel.innerHTML = "<p class=\"text-muted\">" + _channel +"</p>";

    var library = row.insertCell(4);
    library.innerHTML = "<p class=\"text-muted\">" + _library +"</p>";

    var status = row.insertCell(5);
    var actions = row.insertCell(6);

    if(_status == "OFF"){
        status.innerHTML = "<p>" + _status + "</p>";
        actions.innerHTML = "\
        <button type=\"button\" class=\"btn btn-outline-success btn-inline\"><i class=\"fas fa-play\"></i></button> \
        <button type=\"button\" class=\"btn btn-link text-muted\"><i class=\"fas fa-sync-alt\"></i></button> \
        <button type=\"button\" class=\"btn btn-link text-muted\"><i class=\"fas fa-stop\"></i></button>"
    }else if (_status == "ON"){
        status.innerHTML = "<p>" + _status + "</p>";
        actions.innerHTML = "\
        <button type=\"button\" class=\"btn btn-link text-muted\"><i class=\"fas fa-play\"></i></button> \
        <button type=\"button\" class=\"btn btn-outline-warning btn-inline\"><i class=\"fas fa-sync-alt\"></i></button> \
        <button type=\"button\" class=\"btn btn-outline-danger btn-inline\"><i class=\"fas fa-stop\"></i></button>"
    }

    // LOG DRAWER
    var hidden_row = table.insertRow(table.rows.length);
    var log = hidden_row.insertCell(0);

    log.id = _name.toLowerCase().replace(' ', '_') + "_log"
    $("#" + log.id.toString()).attr('colspan', 12)
    $("#" + log.id.toString()).attr('class', "zeroPadding")


    log.innerHTML = "\
        <div class=\"collapse out\" id=\"collapse_"+ name_clean +"\"> \
            <div class=\"content\"> \
                ... \
            </div> \
        </div> \
    "

    log.colspan = 12
    log.class = "zeroPadding"
}

add_row("Motor Driver", "Actuator", "cmd_vel", "Subscriber" , "SCMD", "OFF")
add_row("Motion Sensor", "Sensor", "/imu/raw", "Publisher", "LSM9DS1", "ON")
add_row("SLAM", "Container", "-", "-", "hectorSlam", "ON")
