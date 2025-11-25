// Embedded web content for HTTP server

#include <string.h>
#include "lwip/apps/httpd.h"
#include "lwip/apps/fs.h"
#include "lwip/def.h"

// HTML content (minified)
static const char index_html[] = 
"<!DOCTYPE html>"
"<html><head><title>Pybricks Hub</title>"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
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
"</div></div></div>"
"<script>"
"function setMotor(m,d){document.getElementById('m'+m+'v').textContent=d;fetch('/motor?motor='+m+'&duty='+d)}"
"function stopAll(){for(let i=0;i<4;i++){document.getElementById('m'+i).value=0;setMotor(i,0)}}"
"function testAll(){for(let i=0;i<4;i++){setTimeout(()=>{"
"document.getElementById('m'+i).value=5000;setMotor(i,5000);"
"setTimeout(()=>{document.getElementById('m'+i).value=0;setMotor(i,0)},1000)"
"},i*1500)}}"
"setInterval(()=>location.reload(),3000)"
"</script></body></html>";

// For lwIP 2.x, we need to define custom filesystem
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
    // Nothing to free for static content
    (void)file;
}

#endif
