// ESP8266
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP8266mDNS.h>

// Port expander library
#include <PCF8575.h>

// Default page for web server
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
%TITLE%
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="/index.css"></link>
</head>
<body>
  <table width=800 id='table'>
  <tr><td align='right' colspan="%NUM_OUTPUTS%">
  <BR><BR>
  <input type='button' id='advanced_options' onclick='location.href=/c4update' class='float-right submit-button' value='Advanced options'></button>
  <BR>
  </td></tr>
  <tr><td colspan="%NUM_OUTPUTS%">
  <h2><a href='/'><img src="/c4logo.svg" width="50px"></a> NodeMCU Automation %SW_VERSION%</h2>
  <h4>%HOSTNAME%<br>
  Uptime: <span id="uptime">%UPTIME%<span></h4>
  </td></tr>
  %PLACEHOLDER%
  </table>
  <center><p id='blink'>%REBOOT_MESSAGE%</p></center>
  <script src="/c4status.js"></script>
  <script src="/c4clock.js"></script>
  <script src="/c4refresh_gpio.js"></script>
</body>
</html>
)rawliteral";

// WEB page for the update     http://192.168.1.40/c4update
const char c4update_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
%TITLE%
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="/c4config.css"></link>
</head>
<body>
<table width=800>
<tr>
 <td align='right' colspan="3">
  <BR><BR><BR>
 </td>
</tr>
<tr>
 <td colspan="3">
  <h2><a href='/'><img src="https://res.cloudinary.com/control4/image/upload/v1552441579/control4-4ball.svg" width="50px"></a> NodeMCU Automation %SW_VERSION%</h2>
  <h4>%HOSTNAME%<br>
  Uptime: <span id="uptime">%UPTIME%<span></h4>
 </td>
</tr>
<tr>
 <td colspan='1' width='100px'>&nbsp;</td>
 <td align='left'>
 <HR>
 <BR>
 <form action='/apply_config'>
 <b>Connection options:</b><BR><BR>
%WIFI_OPTIONS%
 <BR>
 MAC Address: <input type="text" size="17" name="MAC" id="MrCmdC" value="%MACADDR%"/><BR>
 <HR>
 <BR><BR>
 <b>Advanced Configuration options:</b><BR><BR>
%CONFIG_OPTIONS%
 <BR><BR>
 <BR>
 <HR>
 <BR>
 <b>Output Description:</b><BR><BR>
%CONFIG_DESCRIPTION%
 <BR><BR>
 <input type='reset' value='Reset'>&nbsp;&nbsp;
 <input type='submit' value='Save'>
 </form>
 <BR>
 <hr>
 <BR>
 <b>Firmware update:</b><BR><BR>
  Please select update file:<BR><BR>
  <form method='POST' action='/doUpdate' enctype='multipart/form-data'>
   <input type='file' name='update'><BR><BR>
   <input type='submit' value='Update'>
  </form>
  <BR>
  <hr>
 </td>
 <td colspan='1' width='100px'>&nbsp;</td>
</TR></TABLE>
<script src="/c4clock.js"></script>
<script src="/c4config.js"></script>
</body>
</html>
)rawliteral";

// JavaScript part
const char c4config_js[] PROGMEM = R"rawliteral(

function isHex(char) {
  if (!isNaN(parseInt(char))) {
    return true;
  } else {
    switch (char.toLowerCase()) {
      case "a":
      case "b":
      case "c":
      case "d":
      case "e":
      case "f":
        return true;
        break;
    }
    return false;
  }
}

document.getElementById("MAC").addEventListener('keyup', function() {
  var mac = document.getElementById('MAC').value;
  if (mac.length < 2) {
    return;
  }
  var newMac = mac.replace("-", "");
  if ((isHex(mac[mac.length - 1]) && (isHex(mac[mac.length - 2])))) {
    newMac = newMac + ":";
  }
  document.getElementById('MAC').value = newMac.substring(0,17);
});

)rawliteral";

const char c4status_js[] PROGMEM = R"rawliteral(
function sRly(element) {
  var xhr = new XMLHttpRequest();
  if(element.checked){ xhr.open("GET", "/update?relay="+element.id.split('_')[1]+"&state=1", true); }
  else { xhr.open("GET", "/update?relay="+element.id.split('_')[1]+"&state=0", true); }
  xhr.send();
}
function rCmd(element, command) {
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/update?relay="+element.id.split('_')[1]+"&function="+command, true);
  xhr.send();
}
function tMtr(element) {
  var xhr = new XMLHttpRequest();
  if(element.checked){ xhr.open("GET", "/motor?motor="+element.id.split('_')[1]+"&state=1", true); }
  else { xhr.open("GET", "/motor?motor="+element.id.split('_')[1]+"&state=1", true); }
  xhr.send();
}
function mtr(element, command) {
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/actuator?actuator="+element.id.split('_')[1]+"&function="+command, true);
  xhr.send();
}
function aAngl(element) {
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/actuator?actuator="+(element.id.split('_')[1]-100)+"&function=angle&angle="+element.value, true);
  xhr.send();
}
document.getElementById("advanced_options").onclick = function () {
  location.href = "/c4update";
};
%AUTOMATIC_REBOOT%

)rawliteral";

// CSS style for configuration page
const char c4config_css[] PROGMEM = R"rawliteral(
html {font-family: Arial; display: inline-block; text-align: center;}
h2 {font-size: 3.0rem;}
p {font-size: 3.0rem;}
body {max-width: 1100px; margin:0px auto; padding-bottom: 25px;}

.table-header-rotated {
  border-collapse: collapse;
}
.table-header-rotated td {
  width: 25px;
}
.table-header-rotated th {
  padding: 2px 5px;
}
.table-header-rotated td {
  text-align: center;
  padding: 5px 2px;
  border: 1px solid #ccc;
}
.table-header-rotated th.rotate {
  height: 135px;
  white-space: nowrap;
}
.table-header-rotated th.rotate > div {
  transform: translate(15px, 51px) rotate(315deg);
  width: 25px;
}
.table-header-rotated th.rotate > div > span {
  border-bottom: 1px solid #ccc;
  padding: 5px 10px;
}
.table-header-rotated th.row-header {
  padding: 0 10px;
  border-bottom: 1px solid #ccc;
}

)rawliteral";

// CSS style for configuration page
const char index_css[] PROGMEM = R"rawliteral(
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 3.0rem;}
    p {font-size: 3.0rem;}
    body {max-width: 1100px; margin:0px auto; padding-bottom: 25px;}
    .button {position: relative; display: inline-block; width: 80px; height: 30px} 
    .button input {display: none}
    .switch {position: relative; display: inline-block; width: 80px; height: 30px} 
    .switch input {display: none}
    .slider {position: absolute; top: 0; left: 0; right: 0; bottom: 0; background-color: lightpink; border-radius: 14px}
    .slider:before {position: absolute; content: ""; height: 15px; width: 15px; left: 8px; bottom: 8px; background-color: indianred; -webkit-transition: .4s; transition: .4s; border-radius: 68px}
    input:checked+.slider {background-color: lightgreen}
    input:checked+.slider:before {-webkit-transform: translateX(50px); -ms-transform: translateX(50px); transform: translateX(50px)}
)rawliteral";

// Control4 logo
const char c4logo_svg[] PROGMEM = R"rawliteral(
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 523.69 530.15"><defs><style>.cls-1{fill:#c42032;fill-rule:evenodd;}</style></defs><title>c4logo</title><g id="Layer_2" data-name="Layer 2"><g id="Layer_1-2" data-name="Layer 1"><polygon class="cls-1" points="153.91 338.12 330.57 338.19 330.57 448.27 384.67 448.27 384.67 292.77 214.85 292.87 358.92 93.77 291.56 93.77 141.4 302.53 153.91 338.12"/><path class="cls-1" d="M341.38,519a285.19,285.19,0,0,1-55,4.82c-136.55,0-247.25-110.72-247.25-247.26S149.82,29.26,286.37,29.26c112.36,0,207.23,74.94,237.32,177.58C508.06,137.6,465,74.74,399.18,36.21,273.1-37.6,110.05,5.06,36.21,131.15S5.05,420.27,131.11,494.09C197,532.62,273.33,539.18,341.38,519"/></g></g></svg>
)rawliteral";

// JavaScript - refresh GPIO ports
const char c4clock_js[] PROGMEM = R"rawliteral(
function pad(num, size) {
    var s = "";
    for (i = 0; i < size - num.toString().length; i++)
      s = "0" + s;
    return s + num.toString();
}

function update_clock() {
    var hrs = Math.floor(uptime/3600);
    var min = Math.floor((uptime-(hrs*3600))/60);
    var sec = Math.floor(uptime-(hrs*3600)-(min*60));

    document.getElementById('uptime').innerHTML = pad(hrs, 2) + ":" + pad(min, 2) + ":" + pad(sec, 2);

    setTimeout(update_clock, 950);
    uptime++;
}
setTimeout(function () { update_clock(); }, 1000);
)rawliteral";

// JavaScript - refresh GPIO ports
const char c4refresh_gpio_js[] PROGMEM = R"rawliteral(
var data=""
var utime=0;

function loadXMLDoc(myurl, cb) {
   var xhr = (window.XMLHttpRequest ? new XMLHttpRequest() : new ActiveXObject("Microsoft.XMLHTTP"));
    xhr.onreadystatechange = function() {
        if (xhr.readyState == 4 && xhr.status == 200) {
            if (typeof cb === 'function') cb(xhr.responseText);
        }
    }
   xhr.open("GET", myurl, true);
   xhr.send();
}

function timed_event() {
  var xmlhttp=false;

  loadXMLDoc('/c4get_gpio.txt', function(responseText) {
    const bigObj = JSON.parse(responseText, (key, value, context) => {
      if (key) {
        elm=document.getElementById(key);
        if (elm) {
          if (key.substring(0, 5) == "port_") {
            //console.log("Found 'port_' element " + key + ", set value " + value);            
            elm.checked=value;
          } else if (key == "uptime") {
            //console.log("Found 'uptime' element " + key + ", set value " + value);            
            uptime=value;
          } else if (elm.innerHTML) {
            //console.log("Found 'innerHTML' element " + key + ", set value " + value);            
            elm.innerHTML=value;
          } else {
            //console.log("Found 'default' element " + key + ", set value " + value);            
            elm.value=value;
          }
        }
      }
      return value;
    });
  });
  setTimeout(function () { timed_event(); }, 5000);
}

setTimeout(function () { timed_event(); }, 5000);

)rawliteral";

// JavaScript - Show GPIO ports
const char c4get_gpio[] PROGMEM = R"rawliteral({
%GPIO_PIN_STATES%
})rawliteral";