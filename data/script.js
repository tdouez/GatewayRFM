
/* sav config */
function savButton() {
    console.log("Sav config");
    console.log($("#form_config").serialize());
    $.post('/config.htm', $("#form_config").serialize())
        .done( function(msg, textStatus, xhr) { 
            console.log($("#form_config").serialize());
            $.notify("Saved", "success");
        })
        .fail( function(xhr, textStatus, errorThrown) {
            $.notify("Unable to save "+ errorThrown, "error");
        })
        console.log("Form Submit config");	
}

function decimalToHexString(number)
{
  if (number < 0)
  {
    number = 0xFFFFFFFF + number + 1;
  }

  return number.toString(16).toUpperCase();
}

/* fonction lecture json */
function lire_item_json(myjson) {
    console.log("lire item json");

    let j = JSON.parse(myjson, function(name, value) {
        if (name == "url_mqtt") {
            $("#url_mqtt").val(value);
        }
        if (name == "token_mqtt") {
            $("#token_mqtt").val(value);
        }
        if (name == "port_mqtt") {
            if (value==0) value=1883;
            $("#port_mqtt").val(value);
        }
        if (name == "user_mqtt") {
            $("#user_mqtt").val(value);
        }
        if (name == "pwd_mqtt") {
            $("#pwd_mqtt").val(value);
        }
        if (name == "module_name") {
            $("#module_name").val(value);
        } 
        if (name == "version") {
            document.getElementById("version").innerHTML = value;
        } 
        if (name == "info_config") {
            affiche_info_config(value);
        }
        if (name == "url_post") {
            $("#url_post").val(value);
        }
        if (name == "token_post") {
            $("#token_post").val(value);
        }
    });
}

/* affiche info config sur panel config*/
function affiche_info_config(buffer) {
    document.getElementById("t_info_config").innerHTML = buffer;
    if (buffer == "") {
        document.getElementById("p_info_config").style.display = "none";
    }
    else {
        document.getElementById("p_info_config").style.display = "block";
    }
}

/* mise a jour info */
function maj_infos() { 
    console.log("maj infos");
        
    $.getJSON("/info.json", function(data) {
        let myNewJSON = JSON.stringify(data, null, '\t'); 
        lire_item_json(myNewJSON); 
    })
    .fail(function(xhr, textStatus, errorThrown) {
        console.log( "error " + errorThrown );
    });
}

/* init */
function init () {
    console.log("get config.json");
    
    $.getJSON("/config.json", function(data) {
        let myNewJSON = JSON.stringify(data, null, '\t'); 
        lire_item_json(myNewJSON);     
    })
    .fail(function(xhr, textStatus, errorThrown) {
        console.log( "error " + errorThrown );
    });
}

window.onload=init();
/*timer = window.setInterval("maj_infos()", 5000);*/
