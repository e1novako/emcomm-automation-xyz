#pragma once
#include <Arduino.h>

namespace c4matrix {
static const char MATRIX_PAGE_STYLE[] PROGMEM =
    "<style>body{font:16px Arial,sans-serif;max-width:760px;margin:24px auto;"
    "padding:0 14px;color:#222}fieldset{margin:16px 0;padding:14px;border:1px "
    "solid #bbb}label{display:block;margin:10px 0}input:not([type=checkbox]),"
    "select{box-sizing:border-box;width:100%;padding:8px}input[type=checkbox]{"
    "width:18px;height:18px}button,.button{padding:9px 14px;margin:4px 4px 4px "
    "0}a{color:#075aa5}.status{background:#f3f5f7;padding:12px;white-space:"
    "pre-wrap;overflow-wrap:anywhere}</style>";
static const char HOME_PAGE_SCRIPT[] PROGMEM =
    "<script>async function load(){try{let r=await fetch('/api/status');let "
    "s=await r.json();if(!r.ok)throw Error('Status unavailable');"
    "document.querySelector('#text').value=s.text||'';"
    "document.querySelector('#fill-color').value=s.fillColor}catch(e)"
    "{document.querySelector('#message').textContent='Could not load status.'}}"
    "document.querySelector('#text-form').addEventListener('submit',async e=>{"
    "e.preventDefault();let m=document.querySelector('#message');try{let r=await "
    "fetch('/api/text',{method:'POST',headers:{'Content-Type':'application/"
    "json'},body:JSON.stringify({text:document.querySelector('#text').value})});"
    "let d=await r.json();if(!r.ok)throw Error(d.error||'Request failed');"
    "m.textContent='Text saved to the display.'}catch(x){m.textContent=x.message}"
    "});async function setLeds(command){let m=document.querySelector('#message');"
    "try{let r=await fetch('/api/leds',{method:'POST',headers:{'Content-Type':"
    "'application/json'},body:JSON.stringify(command)});let d=await r.json();"
    "if(!r.ok)throw Error(d.error||'Request failed');"
    "if(command.color)document.querySelector('#fill-color').value="
    "({red:'#ff0000',green:'#00ff00',blue:'#0000ff'})[command.color];"
    "m.textContent='LED settings saved.'}catch(e){m.textContent=e.message}}"
    "document.querySelectorAll('[data-state]').forEach(b=>"
    "b.addEventListener('click',()=>setLeds({state:b.dataset.state})));"
    "document.querySelectorAll('[data-color]').forEach(b=>"
    "b.addEventListener('click',()=>setLeds({color:b.dataset.color})));"
    "document.querySelector('#color-form').addEventListener('submit',e=>{"
    "e.preventDefault();let c=document.querySelector('#fill-color').value;"
    "setLeds({r:parseInt(c.slice(1,3),16),g:parseInt(c.slice(3,5),16),"
    "b:parseInt(c.slice(5,7),16)})});load();</script>";
}
