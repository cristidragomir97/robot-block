
//////////////////////////////////////////
/// UX HELPERS
//////////////////////////////////////////
var logs = [];

function create_id(_name, suffix){
    return _name.toLowerCase().replace(' ', '_') + suffix;
}

function drawer(table, _name, content){
        var endpoint = path + "logs/" +  create_id(_name, "") 

        // LOG DRAWER
        var hidden_row = table.insertRow(table.rows.length);
        var log = hidden_row.insertCell(0);
    
        log.id = _name.toLowerCase().replace(' ', '_') + "_container"
        $("#" + log.id.toString()).attr('colspan', 12)
        $("#" + log.id.toString()).attr('class', "zeroPadding")

        logs.push({
            'id': create_id(_name, "_log"),
            'endpoint': endpoint,
            'name_clean': name_clean
        })
    
        log.innerHTML = "<div class=\"collapse\" id=\"collapse_"+  name_clean + "\"> \
            <div class=\"content\"> \
                <pre class=\"text-muted\" id="+ create_id(_name, "_log") + 
                "></pre> \
            </div> \
        </div>" 

    

}

function action_switches(_name, _status, status, actions){
    var id_base = create_id(_name, "_status")

    if(_status == "OFF"){
        status.innerHTML = "<p class=\"text-muted\">" + _status +"</p>";
        actions.innerHTML = "\
        <button type=\"button\" id=\"" + id_base + "_start\" class=\"btn btn-link text-muted\"><i class=\"fas fa-play\"></i></button> \
        <button type=\"button\" id=\"" + id_base + "_stop\" class=\"btn btn-outline-danger btn-inline\"><i class=\"fas fa-stop\"></i></button>"
    }else if (_status == "ON"){
        status.innerHTML = "<p class=\"text-muted\">" + _status +"</p>";
        actions.innerHTML = "\
        <button type=\"button\" id=\"" + id_base + "_start\" class=\"btn btn-link text-muted\"><i class=\"fas fa-play\"></i></button> \
        <button type=\"button\" id=\"" + id_base + "_stop\" class=\"btn btn-outline-danger btn-inline\"><i class=\"fas fa-stop\"></i></button>"
    }
}

function label(name, _name){
    name.innerHTML = "<p class=\"text-uppercase text-muted font-weight-bold\"><i class=\"fas fa-chevron-up\"></i> &nbsp" + _name  +"</p>"

    // create IDs based on the thread_name
    name_clean = _name.toLowerCase().replace(' ', '_') + "_name"
    name.id = name_clean
    
    $("#" + name_clean).attr('class', 'fixed');
    $("#" + name_clean).attr('data-target','#collapse_'  + name_clean);
    $("#" + name_clean).attr('data-toggle','collapse');
}

function paragraph(obj, label){
    obj.innerHTML = "<p class=\"text-muted\">" + label +"</p>";
}

function clean_table(table){
    var tableHeaderRowCount = 1;
    var table = document.getElementById(table);
    var rowCount = table.rows.length;
    for (var i = tableHeaderRowCount; i < rowCount; i++) {
        table.deleteRow(tableHeaderRowCount);
    }
}

//////////////////////////////////////////
/// TABLE UTILITIES 
//////////////////////////////////////////

function add_devices(_name, _categ, _topic, _channel, _library, _status){
    var table_name = "table_devices"
    var table = document.getElementById(table_name);
    var rows = document.getElementById(table_name).length
    var row = table.insertRow(rows);

    var name = row.insertCell(0);
    var status = row.insertCell(1);
    var actions = row.insertCell(2);
    var categ = row.insertCell(3);
    var topic = row.insertCell(4);
    var channel = row.insertCell(5);
    var library = row.insertCell(6);

    label(name, _name);
    action_switches(_name, _status, status, actions);
    paragraph(categ, _categ);
    paragraph(topic, _topic);
    paragraph(channel, _channel);
    paragraph(library, _library); 

    content = ""
   
    drawer(table, _name );
}

function add_core(_name, _status){
    var table_name = "table_core"
    var table = document.getElementById(table_name);
    var rows = document.getElementById(table_name).length;
    var row = table.insertRow(rows);

    var name = row.insertCell(0);
    var status = row.insertCell(1);
    var actions = row.insertCell(2);
    var _ = row.insertCell(3);
    
     label(name, _name);
    action_switches(_name, _status, status, actions);
    drawer(table, _name)
}

function add_scripts(_name, _command, _status){
    var table_name = "table_scripts"
    var table = document.getElementById(table_name);
    var rows = document.getElementById(table_name).length
    var row = table.insertRow(rows);

    var name = row.insertCell(0);
    var status = row.insertCell(1);
    var actions = row.insertCell(2);
    var command = row.insertCell(3);

    label(name, _name)
    paragraph(command, _command)
    action_switches(_name, _status, status, actions);
    drawer(table, _name)
}

function add_containers(_name, _image, _command, _status){
    console.log(_name, _image, _command, _status)
    var table_name = "table_containers"
    var table = document.getElementById(table_name);
    var rows = document.getElementById(table_name).length
    var row = table.insertRow(rows);

    var name = row.insertCell(0);
    var status = row.insertCell(1);
    var actions = row.insertCell(2);
    var image = row.insertCell(3);
    var command = row.insertCell(4);

    label(name, _name, )
    action_switches(_name, _status, status, actions);
    paragraph(image, _image)
    paragraph(command, _command)
}

//////////////////////////////////////////
/// API 
//////////////////////////////////////////


function get_main(){
    $.ajax(path + "logs/main", {
        success: function(data) {
          data = JSON.parse(data)
          document.getElementById("main_log").innerHTML = data["data"]
          setTimeout(get_main, 500);
        }
    });
}

function get_workers(){
    clean_table("table_devices")
    clean_table("table_scripts")

    $.ajax(path + "workers", {
      success: function(data) {
        data = JSON.parse(data)
        
        jQuery.each(data, function() {
            if (this.hasOwnProperty("command")){
                add_scripts(this['name'], this['command'], "OFF")
            }else{
                add_devices(this['name'], this["categ"], this["topic"], this["role"], this["library"], "OFF")
            }
           
        });
        setTimeout(get_main, 1000);
      },
        error: function() {
         console.log("error bro");
        }
    });
}


function get_logs(){
    try {
        $.each(logs, function() {
            var ui_element = document.getElementById(this["id"])
            $.ajax(this["endpoint"], {
                success: function(data) {
                   
                    data = JSON.parse(data)
                    if(data["data"].length > 0){
                        ui_element.innerHTML = data["data"]
                    } else {
                        ui_element.innerHTML = "Worker not initialised";
                    }
            }});
        });
    } catch (error) {
        console.error(error);
    }

    setTimeout(get_logs, 1000);
}


add_containers("ros-realsense","cristidragomir/ros-realsense", "roslaunch realsense2_camera rs_camera.launch", "ON" )
get_workers()
get_logs()
get_main()