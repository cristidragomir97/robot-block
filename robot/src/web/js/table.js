

function add_row(){
    var table = document.getElementById("table");
    var rows = document.getElementById("table").rows.length;

    var row = table.insertRow(rows);

    var name = row.insertCell(0);
    var categ = row.insertCell(1);
    var topic = row.insertCell(2);
    var channel = row.insertCell(3);
    var library = row.insertCell(4);
    var status = row.insertCell(5);
    var actions = row.insertCell(6);

    name.innerHTML = "<p class=\"text-uppercase font-weight-bold\"><i class=\"fas fa-chevron-up\"></i> &nbsp {NAME}</p>"
}

add_row()

