// Embedded web content for HTTP server with OTA support

#include <string.h>
#include "lwip/apps/httpd.h"
#include "lwip/apps/fs.h"
#include "lwip/def.h"
#include "ota_handler.h"

// HTML content with OTA upload
static const char index_html[] = 
"<!DOCTYPE html>"
"<html><head><title>Pybricks Hub</title>"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<style>"
"body{font-family:Arial;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);min-height:100vh;padding:20px;margin:0}"
".container{max-width:900px;margin:0 auto}"
"h1{color:white;text-align:center;margin-bottom:30px;font-size:2.5em;text-shadow:2px 2px 4px rgba(0,0,0,0.3)}"
".card{background:white;padding:25px;margin:15px 0;border-radius:12px;box-shadow:0 4px 15px rgba(0,0,0,0.2)}"
"h2{color:#667eea;margin-bottom:15px;font-size:1.5em;border-bottom:2px solid #eee;padding-bottom:10px}"
".status{font-size:16px;color:#555;line-height:1.8}"
".motor-control{display:flex;align-items:center;margin:15px 0;padding:10px;background:#f8f9fa;border-radius:8px}"
".motor-control label{font-weight:600;min-width:80px}"
"input[type=range]{flex:1;margin:0 15px;height:8px}"
".value{min-width:70px;text-align:right;font-family:monospace;font-weight:bold;color:#667eea}"
".buttons{display:flex;gap:10px;margin-top:20px}"
"button{flex:1;min-width:120px;background:#667eea;color:white;border:none;padding:12px 24px;border-radius:8px;cursor:pointer;font-size:16px;font-weight:600}"
"button:hover{background:#5568d3}"
".stop{background:#dc3545}.stop:hover{background:#c82333}"
".test{background:#28a745}.test:hover{background:#218838}"
".upload-btn{background:#ff9800}.upload-btn:hover{background:#e68900}"
"input[type=file]{padding:10px;background:#f8f9fa;border-radius:8px;width:100%;margin-bottom:10px}"
"#progress{display:none;height:30px;background:#eee;border-radius:8px;overflow:hidden;margin-top:10px}"
"#progress-bar{height:100%;background:#28a745;transition:width 0.3s;display:flex;align-items:center;justify-content:center;color:white;font-weight:bold}"
"</style></head><body>"
"<div class=\"container\"><h1>🤖 Pybricks Hub</h1>"
"<div class=\"card\"><h2>📊 Status</h2><div class=\"status\"><!--#status--></div></div>"
"<div class=\"card\"><h2>🧭 IMU</h2><div class=\"status\"><!--#imu--></div></div>"
"<div class=\"card\"><h2>⚙️ Motors</h2>"
"<div class=\"motor-control\"><label>Motor 0:</label>"
"<input type=\"range\" id=\"m0\" min=\"-10000\" max=\"10000\" value=\"0\" oninput=\"setMotor(0,this.value)\">"
"<span class=\"value\" id=\"m0v\">0</span></div>"
"<div class=\"motor-control\"><label>Motor 1:</label>"
"<input type=\"range\" id=\"m1\" min=\"-10000\" max=\"10000\" value=\"0\" oninput=\"setMotor(1,this.value)\">"
"<span class=\"value\" id=\"m1v\">0</span></div>"
"<div class=\"motor-control\"><label>Motor 2:</label>"
"<input type=\"range\" id=\"m2\" min=\"-10000\" max=\"10000\" value=\"0\" oninput=\"setMotor(2,this.value)\">"
"<span class=\"value\" id=\"m2v\">0</span></div>"
"<div class=\"motor-control\"><label>Motor 3:</label>"
"<input type=\"range\" id=\"m3\" min=\"-10000\" max=\"10000\" value=\"0\" oninput=\"setMotor(3,this.value)\">"
"<span class=\"value\" id=\"m3v\">0</span></div>"
"<div class=\"buttons\">"
"<button class=\"stop\" onclick=\"stopAll()\">🛑 STOP</button>"
"<button class=\"test\" onclick=\"testAll()\">▶️ Test</button>"
"</div></div>"
"<div class=\"card\"><h2>📦 OTA Firmware Update</h2>"
"<p style=\"color:#666;margin-bottom:15px\">Version: <!--#version--></p>"
"<input type=\"file\" id=\"fwFile\" accept=\".bin\">"
"<button class=\"upload-btn\" onclick=\"uploadFW()\">🚀 Upload & Install</button>"
"<div id=\"progress\"><div id=\"progress-bar\">0%</div></div>"
"<p id=\"status-msg\" style=\"margin-top:10px;color:#666\"></p>"
"</div></div>"
"<script>"
"function setMotor(m,v){document.getElementById('m'+m+'v').textContent=v;fetch('/motor?motor='+m+'&duty='+v);}"
"function stopAll(){for(let i=0;i<4;i++){document.getElementById('m'+i).value=0;setMotor(i,0);}}"
"function testAll(){for(let i=0;i<4;i++){setMotor(i,5000);setTimeout(()=>setMotor(i,0),1000);}}"
"function uploadFW(){"
"const f=document.getElementById('fwFile').files[0];"
"if(!f){alert('Select a .bin file');return;}"
"const fd=new FormData();fd.append('firmware',f);"
"const xhr=new XMLHttpRequest();"
"xhr.upload.onprogress=(e)=>{"
"if(e.lengthComputable){"
"const p=Math.round((e.loaded/e.total)*100);"
"document.getElementById('progress').style.display='block';"
"document.getElementById('progress-bar').style.width=p+'%';"
"document.getElementById('progress-bar').textContent=p+'%';"
"}"
"};"
"xhr.onload=()=>{"
"if(xhr.status==200){"
"document.getElementById('status-msg').textContent='✅ Upload complete! Rebooting in 3s...';"
"setTimeout(()=>location.reload(),5000);"
"}else{"
"document.getElementById('status-msg').textContent='❌ Upload failed: '+xhr.responseText;"
"}"
"};"
"xhr.onerror=()=>document.getElementById('status-msg').textContent='❌ Network error';"
"xhr.open('POST','/upload');"
"xhr.send(fd);"
"}"
"setInterval(()=>location.reload(),30000);"
"</script>"
"</body></html>";

// For lwIP 2.x filesystem
#if LWIP_HTTPD_CUSTOM_FILES

int fs_open_custom(struct fs_file *file, const char *name) {
    if (strcmp(name, "/index.html") == 0 || strcmp(name, "/index.shtml") == 0 || strcmp(name, "/") == 0) {
        file->data = index_html;
        file->len = sizeof(index_html) - 1;
        file->index = 0;
        file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
        return 1;
    }
    return 0;
}

void fs_close_custom(struct fs_file *file) {
    (void)file;
}

#endif
