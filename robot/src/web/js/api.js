
function get_api_path(){
  var path = location.pathname;

  if(path.endsWith("index.html"))
  {
      path = path.substring(0, path.length - "index.html".length);
  }

  if(!path.endsWith("/")) {
      path = path + "/";
  }

  return "http://" + location.host + path + 'api/'
}

path = get_api_path()

function get_config(){
  $.ajax(path + "config", {
      success: function(data) {
        console.log(data)
        document.getElementById("editor").innerHTML = JSON.stringify(data, null, 4);
      },
        error: function() {
         console.log("error bro");
        }
    });
}




function reload(){
  $.ajax(path + "reload", {
    success: function(data) {
      console.log(data)
    },
      error: function() {
       console.log("error bro");
      }
  });
}

function save(){
  contents = sessionToJSON()
  $.post(path + "config", contents)
  console.log(contents) 
}


$("button").click(function() {
  console.log(this.id);
});



get_config()

