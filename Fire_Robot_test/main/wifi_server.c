#include "wifi_server.h"

#include <string.h>

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "flame.h"
#include "relay.h"

#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "freertos/task.h"

// Getters and Setters from main.c
extern void get_robot_position(float *x, float *y, float *heading);
extern void reset_robot_position(void);
extern void set_node_alert(int node_id, bool has_flame, int gas_level);
extern void get_node_status(int node_id, bool *has_flame, int *gas_level, bool *active_alert);
extern bool get_auto_mode(void);
extern int get_robot_state(void);
extern void get_node_config(int id, float *x, float *y, char *name, size_t max_len);
extern void update_node_config(int id, float x, float y, const char *name);

#include <ctype.h>

static void url_decode(char *dst, const char *src) {
    char a, b;
    while (*src) {
        if ((*src == '%') &&
            ((a = src[1]) && (b = src[2])) &&
            (isxdigit((int)a) && isxdigit((int)b))) {
            if (a >= 'a') a -= 'a'-'A';
            if (a >= 'A') a -= ('A' - 10);
            else a -= '0';
            if (b >= 'a') b -= 'a'-'A';
            if (b >= 'A') b -= ('A' - 10);
            else b -= '0';
            *dst++ = 16*a+b;
            src += 3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = '\0';
}

static char s_cam_ip[32] = "";

#define WIFI_AP_SSID "FireRobot"
#define WIFI_AP_PASSWORD "fire12345"
#define WIFI_AP_CHANNEL 1
#define WIFI_AP_MAX_CONN 4

static const char *TAG = "WIFI_SERVER";
static wifi_server_command_cb_t s_command_cb;

static const char INDEX_HTML[] =
    "<!DOCTYPE html>\n"
    "<html>\n"
    "<head>\n"
    "    <meta charset=\"utf-8\">\n"
    "    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
    "    <title>Fire Fighting Robot - 3D Dashboard</title>\n"
    "    <style>\n"
    "        :root {\n"
    "            --bg-color: #0d1117;\n"
    "            --card-bg: #161b22;\n"
    "            --border-color: #30363d;\n"
    "            --text-color: #f0f6fc;\n"
    "            --text-muted: #8b949e;\n"
    "            --accent-green: #238636;\n"
    "            --accent-green-hover: #2ea043;\n"
    "            --accent-red: #da3633;\n"
    "            --accent-red-hover: #f85149;\n"
    "            --accent-blue: #1f6feb;\n"
    "            --accent-orange: #d29922;\n"
    "        }\n"
    "\n"
    "        body {\n"
    "            font-family: -apple-system, BlinkMacSystemFont, \"Segoe UI\", Helvetica, Arial, sans-serif;\n"
    "            margin: 0;\n"
    "            padding: 0;\n"
    "            background-color: var(--bg-color);\n"
    "            color: var(--text-color);\n"
    "            overflow-x: hidden;\n"
    "        }\n"
    "\n"
    "        header {\n"
    "            background-color: var(--card-bg);\n"
    "            border-bottom: 1px solid var(--border-color);\n"
    "            padding: 10px 20px;\n"
    "            display: flex;\n"
    "            justify-content: space-between;\n"
    "            align-items: center;\n"
    "        }\n"
    "\n"
    "        h1 {\n"
    "            margin: 0;\n"
    "            font-size: 20px;\n"
    "            font-weight: 600;\n"
    "            display: flex;\n"
    "            align-items: center;\n"
    "            gap: 10px;\n"
    "        }\n"
    "\n"
    "        .status-dot {\n"
    "            width: 10px;\n"
    "            height: 10px;\n"
    "            border-radius: 50%;\n"
    "            background-color: var(--accent-red);\n"
    "            display: inline-block;\n"
    "            box-shadow: 0 0 8px var(--accent-red);\n"
    "            transition: all 0.3s ease;\n"
    "        }\n"
    "\n"
    "        .status-dot.connected {\n"
    "            background-color: #39d353;\n"
    "            box-shadow: 0 0 8px #39d353;\n"
    "        }\n"
    "\n"
    "        .ip-config {\n"
    "            display: flex;\n"
    "            gap: 10px;\n"
    "            align-items: center;\n"
    "        }\n"
    "\n"
    "        .ip-config input {\n"
    "            background: var(--bg-color);\n"
    "            border: 1px solid var(--border-color);\n"
    "            color: var(--text-color);\n"
    "            padding: 6px 12px;\n"
    "            border-radius: 6px;\n"
    "            font-size: 14px;\n"
    "            width: 130px;\n"
    "        }\n"
    "\n"
    "        .btn {\n"
    "            background-color: var(--accent-blue);\n"
    "            color: white;\n"
    "            border: none;\n"
    "            padding: 6px 16px;\n"
    "            font-size: 14px;\n"
    "            font-weight: 500;\n"
    "            border-radius: 6px;\n"
    "            cursor: pointer;\n"
    "            transition: opacity 0.2s;\n"
    "        }\n"
    "\n"
    "        .btn:hover {\n"
    "            opacity: 0.9;\n"
    "        }\n"
    "\n"
    "        .dashboard-grid {\n"
    "            display: grid;\n"
    "            grid-template-columns: 350px 1fr 350px;\n"
    "            gap: 15px;\n"
    "            padding: 15px;\n"
    "            height: calc(100vh - 65px);\n"
    "            box-sizing: border-box;\n"
    "        }\n"
    "\n"
    "        @media (max-width: 1200px) {\n"
    "            .dashboard-grid {\n"
    "                grid-template-columns: 1fr;\n"
    "                height: auto;\n"
    "            }\n"
    "        }\n"
    "\n"
    "        .panel {\n"
    "            background-color: var(--card-bg);\n"
    "            border: 1px solid var(--border-color);\n"
    "            border-radius: 8px;\n"
    "            display: flex;\n"
    "            flex-direction: column;\n"
    "            overflow: hidden;\n"
    "            position: relative;\n"
    "        }\n"
    "\n"
    "        .panel-header {\n"
    "            background-color: rgba(240, 246, 252, 0.03);\n"
    "            border-bottom: 1px solid var(--border-color);\n"
    "            padding: 10px 15px;\n"
    "            font-size: 14px;\n"
    "            font-weight: 600;\n"
    "            display: flex;\n"
    "            justify-content: space-between;\n"
    "            align-items: center;\n"
    "        }\n"
    "\n"
    "        .panel-body {\n"
    "            padding: 15px;\n"
    "            flex-grow: 1;\n"
    "            overflow-y: auto;\n"
    "        }\n"
    "\n"
    "        /* Video Streaming */\n"
    "        .video-container {\n"
    "            width: 100%;\n"
    "            height: 220px;\n"
    "            background: #000;\n"
    "            position: relative;\n"
    "            display: flex;\n"
    "            align-items: center;\n"
    "            justify-content: center;\n"
    "            border-radius: 6px;\n"
    "            overflow: hidden;\n"
    "        }\n"
    "\n"
    "        .video-stream {\n"
    "            width: 100%;\n"
    "            height: 100%;\n"
    "            object-fit: cover;\n"
    "        }\n"
    "\n"
    "        .video-placeholder {\n"
    "            color: var(--text-muted);\n"
    "            font-size: 14px;\n"
    "            text-align: center;\n"
    "        }\n"
    "\n"
    "        /* Controls Grid */\n"
    "        .control-pad {\n"
    "            display: grid;\n"
    "            grid-template-columns: repeat(3, 1fr);\n"
    "            gap: 10px;\n"
    "            max-width: 200px;\n"
    "            margin: 15px auto;\n"
    "        }\n"
    "\n"
    "        .ctrl-btn {\n"
    "            height: 50px;\n"
    "            border: 1px solid var(--border-color);\n"
    "            background: var(--bg-color);\n"
    "            color: var(--text-color);\n"
    "            border-radius: 8px;\n"
    "            font-weight: 600;\n"
    "            cursor: pointer;\n"
    "            transition: all 0.2s;\n"
    "            display: flex;\n"
    "            align-items: center;\n"
    "            justify-content: center;\n"
    "            font-size: 18px;\n"
    "        }\n"
    "\n"
    "        .ctrl-btn:hover {\n"
    "            background: rgba(240, 246, 252, 0.05);\n"
    "            border-color: var(--text-muted);\n"
    "        }\n"
    "\n"
    "        .ctrl-btn.active {\n"
    "            background: var(--accent-blue);\n"
    "            color: white;\n"
    "            border-color: var(--accent-blue);\n"
    "        }\n"
    "\n"
    "        .ctrl-btn.stop {\n"
    "            background-color: var(--accent-red);\n"
    "            color: white;\n"
    "            border-color: var(--accent-red);\n"
    "        }\n"
    "\n"
    "        .ctrl-btn.stop:hover {\n"
    "            background-color: var(--accent-red-hover);\n"
    "        }\n"
    "\n"
    "        /* Telemetry */\n"
    "        .telemetry-row {\n"
    "            display: flex;\n"
    "            justify-content: space-between;\n"
    "            padding: 8px 0;\n"
    "            border-bottom: 1px solid rgba(48, 54, 61, 0.5);\n"
    "            font-size: 14px;\n"
    "        }\n"
    "\n"
    "        .telemetry-row:last-child {\n"
    "            border-bottom: none;\n"
    "        }\n"
    "\n"
    "        .telemetry-val {\n"
    "            font-weight: 600;\n"
    "        }\n"
    "\n"
    "        .telemetry-val.on { color: var(--accent-red); }\n"
    "        .telemetry-val.off { color: #39d353; }\n"
    "\n"
    "        /* Sensor Nodes */\n"
    "        .node-card {\n"
    "            border: 1px solid var(--border-color);\n"
    "            border-radius: 6px;\n"
    "            background: rgba(240, 246, 252, 0.01);\n"
    "            padding: 10px;\n"
    "            margin-bottom: 12px;\n"
    "            transition: border-color 0.3s;\n"
    "        }\n"
    "\n"
    "        .node-card.alert {\n"
    "            border-color: var(--accent-red);\n"
    "            background: rgba(218, 54, 51, 0.05);\n"
    "        }\n"
    "\n"
    "        .node-card-header {\n"
    "            display: flex;\n"
    "            justify-content: space-between;\n"
    "            align-items: center;\n"
    "            font-weight: 600;\n"
    "            font-size: 14px;\n"
    "            margin-bottom: 8px;\n"
    "        }\n"
    "\n"
    "        .node-status {\n"
    "            font-size: 12px;\n"
    "            padding: 2px 6px;\n"
    "            border-radius: 10px;\n"
    "            background: var(--border-color);\n"
    "        }\n"
    "\n"
    "        .node-card.alert .node-status {\n"
    "            background: var(--accent-red);\n"
    "            color: white;\n"
    "            animation: pulse 1.5s infinite;\n"
    "        }\n"
    "\n"
    "        /* Progress meter */\n"
    "        .gas-meter {\n"
    "            height: 8px;\n"
    "            background: #0d1117;\n"
    "            border-radius: 4px;\n"
    "            overflow: hidden;\n"
    "            margin-top: 4px;\n"
    "        }\n"
    "\n"
    "        .gas-fill {\n"
    "            height: 100%;\n"
    "            width: 0%;\n"
    "            background: #39d353;\n"
    "            transition: width 0.3s ease, background-color 0.3s;\n"
    "        }\n"
    "\n"
    "        /* Logs Panel */\n"
    "        .logs-container {\n"
    "            font-family: \"Courier New\", Courier, monospace;\n"
    "            font-size: 12px;\n"
    "            background: #0d1117;\n"
    "            border: 1px solid var(--border-color);\n"
    "            padding: 10px;\n"
    "            border-radius: 6px;\n"
    "            height: 160px;\n"
    "            overflow-y: auto;\n"
    "            color: #39d353;\n"
    "            line-height: 1.4;\n"
    "        }\n"
    "\n"
    "        /* 3D Map Area */\n"
    "        #canvas3d {\n"
    "            width: 100%;\n"
    "            height: 100%;\n"
    "            display: block;\n"
    "        }\n"
    "\n"
    "        /* OTA Panel in Dashboard */\n"
    "        .ota-section {\n"
    "            margin-top: 15px;\n"
    "            border-top: 1px solid var(--border-color);\n"
    "            padding-top: 15px;\n"
    "        }\n"
    "\n"
    "        .progress-container {\n"
    "            background-color: var(--bg-color);\n"
    "            border-radius: 6px;\n"
    "            height: 10px;\n"
    "            margin: 10px 0;\n"
    "            overflow: hidden;\n"
    "            display: none;\n"
    "        }\n"
    "\n"
    "        .progress-bar {\n"
    "            height: 100%;\n"
    "            width: 0%;\n"
    "            background-color: #39d353;\n"
    "            transition: width 0.1s ease;\n"
    "        }\n"
    "\n"
    "        @keyframes pulse {\n"
    "            0% { opacity: 0.6; }\n"
    "            50% { opacity: 1; }\n"
    "            100% { opacity: 0.6; }\n"
    "        }\n"
    "\n"
    "        @keyframes pulse-ring {\n"
    "            0% { transform: scale(0.9); opacity: 0.8; }\n"
    "            100% { transform: scale(1.4); opacity: 0; }\n"
    "        }\n"
    "\n"
    "        /* Tab Navigation Bar */\n"
    "        .nav-tabs {\n"
    "            display: flex;\n"
    "            background-color: var(--card-bg);\n"
    "            border-bottom: 1px solid var(--border-color);\n"
    "            padding: 0 20px;\n"
    "        }\n"
    "\n"
    "        .tab-btn {\n"
    "            background: none;\n"
    "            border: none;\n"
    "            color: var(--text-muted);\n"
    "            padding: 14px 20px;\n"
    "            font-size: 14px;\n"
    "            font-weight: 500;\n"
    "            cursor: pointer;\n"
    "            border-bottom: 2px solid transparent;\n"
    "            transition: color 0.2s, border-bottom-color 0.2s;\n"
    "            font-family: inherit;\n"
    "        }\n"
    "\n"
    "        .tab-btn:hover {\n"
    "            color: var(--text-color);\n"
    "        }\n"
    "\n"
    "        .tab-btn.active {\n"
    "            color: var(--accent-orange);\n"
    "            border-bottom-color: var(--accent-orange);\n"
    "        }\n"
    "\n"
    "        /* Tab Content Panel */\n"
    "        .tab-content {\n"
    "            display: none;\n"
    "        }\n"
    "\n"
    "        .tab-content.active {\n"
    "            display: block;\n"
    "        }\n"
    "\n"
    "        /* Training Panel Layout */\n"
    "        .training-grid {\n"
    "            display: grid;\n"
    "            grid-template-columns: 1fr 1.2fr;\n"
    "            gap: 20px;\n"
    "            padding: 20px;\n"
    "            max-width: 1400px;\n"
    "            margin: 0 auto;\n"
    "        }\n"
    "\n"
    "        .training-card {\n"
    "            background-color: var(--card-bg);\n"
    "            border: 1px solid var(--border-color);\n"
    "            border-radius: 8px;\n"
    "            padding: 20px;\n"
    "            margin-bottom: 20px;\n"
    "        }\n"
    "\n"
    "        .training-card h3 {\n"
    "            margin-top: 0;\n"
    "            margin-bottom: 12px;\n"
    "            font-size: 16px;\n"
    "            color: var(--accent-orange);\n"
    "            border-bottom: 1px solid var(--border-color);\n"
    "            padding-bottom: 8px;\n"
    "        }\n"
    "\n"
    "        .training-step {\n"
    "            margin-bottom: 15px;\n"
    "            display: flex;\n"
    "            align-items: flex-start;\n"
    "        }\n"
    "\n"
    "        .training-step-num {\n"
    "            display: inline-block;\n"
    "            background-color: var(--accent-orange);\n"
    "            color: #0d1117;\n"
    "            min-width: 20px;\n"
    "            height: 20px;\n"
    "            border-radius: 50%;\n"
    "            text-align: center;\n"
    "            font-weight: bold;\n"
    "            font-size: 12px;\n"
    "            line-height: 20px;\n"
    "            margin-right: 8px;\n"
    "        }\n"
    "\n"
    "        .training-step-desc {\n"
    "            font-size: 13px;\n"
    "            color: var(--text-color);\n"
    "            line-height: 1.4;\n"
    "        }\n"
    "\n"
    "        .math-block {\n"
    "            background-color: var(--bg-color);\n"
    "            border-left: 3px solid var(--accent-blue);\n"
    "            padding: 10px 15px;\n"
    "            font-family: 'Courier New', Courier, monospace;\n"
    "            font-size: 13px;\n"
    "            margin: 12px 0;\n"
    "            border-radius: 0 4px 4px 0;\n"
    "            overflow-x: auto;\n"
    "            color: var(--text-color);\n"
    "        }\n"
    "    </style>\n"
    "    <!-- Three.js and OrbitControls Libraries -->\n"
    "    <script src=\"https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js\"></script>\n"
    "    <script src=\"https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js\"></script>\n"
    "</head>\n"
    "<body>\n"
    "\n"
    "    <header>\n"
    "        <h1>\n"
    "            <span id=\"conn-dot\" class=\"status-dot\"></span>\n"
    "            Fire Fighting Robot - 3D Command Dashboard\n"
    "        </h1>\n"
    "        <div class=\"ip-config\">\n"
    "            <span style=\"font-size: 14px; color: var(--text-muted);\">Robot IP:</span>\n"
    "            <input type=\"text\" id=\"robot-ip\" value=\"192.168.4.1\">\n"
    "            <button class=\"btn\" id=\"connect-btn\" onclick=\"toggleConnect()\">Connect</button>\n"
    "        </div>\n"
    "    </header>\n"
    "\n"
    "    <nav class=\"nav-tabs\">\n"
    "        <button class=\"tab-btn active\" id=\"tab-btn-command\" onclick=\"switchTab('command-center')\">Command Center</button>\n"
    "        <button class=\"tab-btn\" id=\"tab-btn-calibration\" onclick=\"switchTab('calibration-training')\">Training & Calibration</button>\n"
    "        <button class=\"tab-btn\" id=\"tab-btn-firmware\" onclick=\"switchTab('firmware-update')\">Firmware Update</button>\n"
    "    </nav>\n"
    "\n"
    "    <!-- Tab 1: Command Center -->\n"
    "    <div id=\"tab-command-center\" class=\"tab-content active\">\n"
    "        <div class=\"dashboard-grid\">\n"
    "        <!-- Left Column: Video stream, Manual controls -->\n"
    "        <div class=\"panel\">\n"
    "            <div class=\"panel-header\">Live Feed & Navigation Control</div>\n"
    "            <div class=\"panel-body\">\n"
    "                <div class=\"video-container\">\n"
    "                    <img id=\"stream-img\" class=\"video-stream\" src=\"\" style=\"display:none\">\n"
    "                    <div id=\"video-placeholder\" class=\"video-placeholder\">\n"
    "                        ESP32-CAM Not Streaming<br>\n"
    "                        <span style=\"font-size:12px;color:var(--text-muted)\">(IP Address unknown or camera offline)</span>\n"
    "                    </div>\n"
    "                </div>\n"
    "\n"
    "                <div style=\"margin-top: 15px; text-align: center;\">\n"
    "                    <span style=\"font-size:13px; color: var(--text-muted);\">Drive Mode:</span>\n"
    "                    <button class=\"btn\" id=\"mode-btn\" onclick=\"toggleMode()\" style=\"background-color:var(--accent-orange); margin-left:10px;\">Switch to Manual</button>\n"
    "                </div>\n"
    "\n"
    "                <div style=\"margin-top: 12px; background: var(--card-bg); padding: 10px; border-radius: 6px; border: 1px solid var(--border-color);\">\n"
    "                    <div style=\"display: flex; justify-content: space-between; font-size: 12px; margin-bottom: 6px;\">\n"
    "                        <span style=\"color: var(--text-muted);\">Motor Speed Limit:</span>\n"
    "                        <span id=\"speed-val-display\" style=\"font-weight: 600; color: var(--accent-orange);\">70%</span>\n"
    "                    </div>\n"
    "                    <input type=\"range\" id=\"motor-speed-slider\" min=\"30\" max=\"100\" value=\"70\" style=\"width: 100%; cursor: pointer;\" oninput=\"onSpeedSliderInput(this.value)\" onchange=\"onSpeedSliderChange(this.value)\">\n"
    "                </div>\n"
    "\n"
    "                <div class=\"control-pad\">\n"
    "                    <span></span>\n"
    "                    <button class=\"ctrl-btn\" onclick=\"sendCmd('forward')\">▲</button>\n"
    "                    <span></span>\n"
    "                    <button class=\"ctrl-btn\" onclick=\"sendCmd('left')\">◀</button>\n"
    "                    <button class=\"ctrl-btn stop\" onclick=\"sendCmd('stop')\">■</button>\n"
    "                    <button class=\"ctrl-btn\" onclick=\"sendCmd('right')\">▶</button>\n"
    "                    <span></span>\n"
    "                    <button class=\"ctrl-btn\" onclick=\"sendCmd('reverse')\">▼</button>\n"
    "                    <span></span>\n"
    "                </div>\n"
    "\n"
    "                <div style=\"display:grid; grid-template-columns:1fr 1fr; gap:10px; margin-top:10px;\">\n"
    "                    <button class=\"btn\" style=\"background-color:var(--accent-red);\" onclick=\"sendCmd('pump_on')\">Pump ON</button>\n"
    "                    <button class=\"btn\" style=\"background-color:var(--border-color); color:var(--text-color);\" onclick=\"sendCmd('pump_off')\">Pump OFF</button>\n"
    "                    <button class=\"btn\" style=\"background-color:var(--accent-blue); grid-column:1/3;\" onclick=\"sendCmd('sweep')\">Sweep Extinguish Nozzle</button>\n"
    "                </div>\n"
    "            </div>\n"
    "        </div>\n"
    "\n"
    "        <!-- Middle Column: 3D Tracking view -->\n"
    "        <div class=\"panel\">\n"
    "            <div class=\"panel-header\">\n"
    "                <span>Real-Time 3D Position Tracking</span>\n"
    "                <span id=\"coords-display\" style=\"font-size:12px; color:var(--text-muted);\">X: 0.00m, Y: 0.00m, H: 0.0°</span>\n"
    "            </div>\n"
    "            <div style=\"flex-grow:1; background:#05070a; position:relative;\">\n"
    "                <div id=\"canvas3d\"></div>\n"
    "            </div>\n"
    "        </div>\n"
    "\n"
    "        <!-- Right Column: Telemetry, Sensor nodes, Logs -->\n"
    "        <div class=\"panel\">\n"
    "            <div class=\"panel-header\">System Health & Remote Telemetry</div>\n"
    "            <div class=\"panel-body\">\n"
    "                <div class=\"telemetry-row\">\n"
    "                    <span>Robot Mode</span>\n"
    "                    <span id=\"robot-mode\" class=\"telemetry-val\">STANDBY</span>\n"
    "                </div>\n"
    "                <div class=\"telemetry-row\">\n"
    "                    <span>Current FSM State</span>\n"
    "                    <span id=\"robot-state\" class=\"telemetry-val\">IDLE</span>\n"
    "                </div>\n"
    "                <div class=\"telemetry-row\">\n"
    "                    <span>Pump Status</span>\n"
    "                    <span id=\"telemetry-pump\" class=\"telemetry-val off\">OFF</span>\n"
    "                </div>\n"
    "                <div class=\"telemetry-row\">\n"
    "                    <span>Flame Local Sensors</span>\n"
    "                    <span id=\"telemetry-flame\" class=\"telemetry-val\">L: -- | F: -- | R: --</span>\n"
    "                </div>\n"
    "\n"
    "                <div style=\"margin-top:15px; margin-bottom:5px; font-weight:600; font-size:13px;\">Remote Sensor Nodes</div>\n"
    "                \n"
    "                <div class=\"node-card\" id=\"node-card-1\"></div>\n"
    "                <div class=\"node-card\" id=\"node-card-2\"></div>\n"
    "\n"
    "                <div style=\"margin-top:15px; margin-bottom:5px; font-weight:600; font-size:13px;\">Event Logs</div>\n"
    "                <div class=\"logs-container\" id=\"logs-box\"></div>\n"
    "\n"
    "                <div class=\"ota-section\" style=\"margin-top:15px; border-top:1px solid var(--border-color); padding-top:15px;\">\n"
    "                    <div style=\"font-weight:600; font-size:13px; margin-bottom:8px;\">Odometer Calibration & Training</div>\n"
    "                    <div style=\"display:grid; grid-template-columns:1fr 1fr; gap:8px; margin-bottom:8px;\">\n"
    "                        <div>\n"
    "                            <label style=\"font-size:11px; color:var(--text-muted); display:block;\">Travel Speed (m/s):</label>\n"
    "                            <input type=\"number\" step=\"0.01\" id=\"cal-speed-input\" value=\"0.20\" style=\"width:100%; padding:6px 8px; background:var(--bg-color); border:1px solid var(--border-color); color:var(--text-color); border-radius:4px; margin-top:2px; box-sizing:border-box;\">\n"
    "                        </div>\n"
    "                        <div>\n"
    "                            <label style=\"font-size:11px; color:var(--text-muted); display:block;\">Turn Rate (rad/s):</label>\n"
    "                            <input type=\"number\" step=\"0.01\" id=\"cal-turn-input\" value=\"1.50\" style=\"width:100%; padding:6px 8px; background:var(--bg-color); border:1px solid var(--border-color); color:var(--text-color); border-radius:4px; margin-top:2px; box-sizing:border-box;\">\n"
    "                        </div>\n"
    "                    </div>\n"
    "                    <button class=\"btn\" style=\"width:100%; font-size:13px; background-color:var(--accent-orange); margin-bottom:0;\" onclick=\"saveCalibration()\">Save Odom Calibration</button>\n"
    "                </div>\n"
    "            </div>\n"
    "        </div>\n"
    "    </div>\n"
    "    </div>\n"
    "\n"
    "    <!-- Tab 2: Training & Calibration -->\n"
    "    <div id=\"tab-calibration-training\" class=\"tab-content\">\n"
    "        <div class=\"training-grid\">\n"
    "            \n"
    "            <!-- Left Column: Training Stats and manual controls -->\n"
    "            <div>\n"
    "                <div class=\"training-card\">\n"
    "                    <h3>Odometer Status</h3>\n"
    "                    <div style=\"display:flex; justify-content:space-between; margin-bottom:12px; font-size:14px;\">\n"
    "                        <span>Estimated X Coord:</span>\n"
    "                        <span id=\"train-x-val\" style=\"font-weight:600; color:var(--accent-blue);\">0.00m</span>\n"
    "                    </div>\n"
    "                    <div style=\"display:flex; justify-content:space-between; margin-bottom:12px; font-size:14px;\">\n"
    "                        <span>Estimated Y Coord:</span>\n"
    "                        <span id=\"train-y-val\" style=\"font-weight:600; color:var(--accent-blue);\">0.00m</span>\n"
    "                    </div>\n"
    "                    <div style=\"display:flex; justify-content:space-between; margin-bottom:20px; font-size:14px;\">\n"
    "                        <span>Estimated Heading:</span>\n"
    "                        <span id=\"train-h-val\" style=\"font-weight:600; color:var(--accent-blue);\">0.0°</span>\n"
    "                    </div>\n"
    "                    <button class=\"btn\" style=\"width:100%; background-color:var(--accent-red); margin-bottom:0;\" onclick=\"resetOdometer()\">Reset Odometer to Home (0,0)</button>\n"
    "                </div>\n"
    "\n"
    "                <div class=\"training-card\">\n"
    "                    <h3>Manual Drive & Calibration Control</h3>\n"
    "                    <div style=\"margin-bottom: 15px; text-align: center;\">\n"
    "                        <span style=\"font-size:13px; color: var(--text-muted);\">Drive Mode:</span>\n"
    "                        <button class=\"btn\" id=\"mode-btn-tab\" onclick=\"toggleMode()\" style=\"background-color:var(--accent-orange); margin-left:10px;\">Switch to Manual</button>\n"
    "                    </div>\n"
    "\n"
    "                    <div style=\"margin-bottom: 12px; background: var(--card-bg); padding: 10px; border-radius: 6px; border: 1px solid var(--border-color);\">\n"
    "                        <div style=\"display: flex; justify-content: space-between; font-size: 12px; margin-bottom: 6px;\">\n"
    "                            <span style=\"color: var(--text-muted);\">Motor Speed Limit:</span>\n"
    "                            <span id=\"speed-val-display-tab\" style=\"font-weight: 600; color: var(--accent-orange);\">70%</span>\n"
    "                        </div>\n"
    "                        <input type=\"range\" id=\"motor-speed-slider-tab\" min=\"30\" max=\"100\" value=\"70\" style=\"width: 100%; cursor: pointer;\" oninput=\"onSpeedSliderInputTab(this.value)\" onchange=\"onSpeedSliderChangeTab(this.value)\">\n"
    "                    </div>\n"
    "\n"
    "                    <div class=\"control-pad\">\n"
    "                        <span></span>\n"
    "                        <button class=\"ctrl-btn\" onclick=\"sendCmd('forward')\">▲</button>\n"
    "                        <span></span>\n"
    "                        <button class=\"ctrl-btn\" onclick=\"sendCmd('left')\">◀</button>\n"
    "                        <button class=\"ctrl-btn stop\" onclick=\"sendCmd('stop')\">■</button>\n"
    "                        <button class=\"ctrl-btn\" onclick=\"sendCmd('right')\">▶</button>\n"
    "                        <span></span>\n"
    "                        <button class=\"ctrl-btn\" onclick=\"sendCmd('reverse')\">▼</button>\n"
    "                        <span></span>\n"
    "                    </div>\n"
    "                </div>\n"
    "            </div>\n"
    "\n"
    "            <!-- Right Column: Calibration settings and node positioning -->\n"
    "            <div>\n"
    "                <div class=\"training-card\">\n"
    "                    <h3>Calibration Settings</h3>\n"
    "                    <div style=\"display:grid; grid-template-columns:1fr 1fr; gap:10px; margin-bottom:15px;\">\n"
    "                        <div>\n"
    "                            <label style=\"font-size:12px; color:var(--text-muted); display:block;\">Travel Speed (m/s):</label>\n"
    "                            <input type=\"number\" step=\"0.01\" id=\"cal-speed-input-tab\" value=\"0.20\" style=\"width:100%; padding:8px 12px; background:var(--bg-color); border:1px solid var(--border-color); color:var(--text-color); border-radius:4px; margin-top:4px; box-sizing:border-box;\">\n"
    "                        </div>\n"
    "                        <div>\n"
    "                            <label style=\"font-size:12px; color:var(--text-muted); display:block;\">Turn Rate (rad/s):</label>\n"
    "                            <input type=\"number\" step=\"0.01\" id=\"cal-turn-input-tab\" value=\"1.50\" style=\"width:100%; padding:8px 12px; background:var(--bg-color); border:1px solid var(--border-color); color:var(--text-color); border-radius:4px; margin-top:4px; box-sizing:border-box;\">\n"
    "                        </div>\n"
    "                    </div>\n"
    "                    <button class=\"btn\" style=\"width:100%; background-color:var(--accent-orange); margin-bottom:0;\" onclick=\"saveCalibrationTab()\">Save Calibration Constants</button>\n"
    "                </div>\n"
    "\n"
    "                <div class=\"training-card\">\n"
    "                    <h3>Node Position Training & Editing</h3>\n"
    "                    <p style=\"font-size:12px; color:var(--text-muted); margin-bottom:12px; line-height:1.4;\">\n"
    "                        Select a node to configure. You can manually edit values or drive the robot to the node and click <em>Capture Robot Location</em>.\n"
    "                    </p>\n"
    "                    <div style=\"margin-bottom: 12px;\">\n"
    "                        <label style=\"font-size:12px; color:var(--text-muted); display:block;\">Select Node:</label>\n"
    "                        <select id=\"train-node-select\" onchange=\"onTrainNodeSelectChange(this.value)\" style=\"width:100%; padding:8px; background:var(--bg-color); border:1px solid var(--border-color); color:var(--text-color); border-radius:4px; margin-top:4px; box-sizing:border-box; font-family:inherit;\">\n"
    "                            <option value=\"1\">Node 1</option>\n"
    "                            <option value=\"2\">Node 2</option>\n"
    "                        </select>\n"
    "                    </div>\n"
    "                    \n"
    "                    <div style=\"margin-bottom: 12px;\">\n"
    "                        <label style=\"font-size:12px; color:var(--text-muted); display:block;\">Node Name:</label>\n"
    "                        <input type=\"text\" id=\"train-node-name\" value=\"Node 1\" style=\"width:100%; padding:8px; background:var(--bg-color); border:1px solid var(--border-color); color:var(--text-color); border-radius:4px; margin-top:4px; box-sizing:border-box; font-family:inherit;\">\n"
    "                    </div>\n"
    "\n"
    "                    <div style=\"display:grid; grid-template-columns:1fr 1fr; gap:10px; margin-bottom:15px;\">\n"
    "                        <div>\n"
    "                            <label style=\"font-size:12px; color:var(--text-muted); display:block;\">X Coordinate (m):</label>\n"
    "                            <input type=\"number\" step=\"0.01\" id=\"train-node-x\" value=\"1.50\" style=\"width:100%; padding:8px; background:var(--bg-color); border:1px solid var(--border-color); color:var(--text-color); border-radius:4px; margin-top:4px; box-sizing:border-box; font-family:inherit;\">\n"
    "                        </div>\n"
    "                        <div>\n"
    "                            <label style=\"font-size:12px; color:var(--text-muted); display:block;\">Y Coordinate (m):</label>\n"
    "                            <input type=\"number\" step=\"0.01\" id=\"train-node-y\" value=\"0.00\" style=\"width:100%; padding:8px; background:var(--bg-color); border:1px solid var(--border-color); color:var(--text-color); border-radius:4px; margin-top:4px; box-sizing:border-box; font-family:inherit;\">\n"
    "                        </div>\n"
    "                    </div>\n"
    "\n"
    "                    <div style=\"display:grid; grid-template-columns:1.2fr 1fr; gap:10px;\">\n"
    "                        <button class=\"btn\" style=\"margin-bottom:0; width:100%; background-color:var(--accent-blue);\" onclick=\"captureNodeLocation()\">Capture Robot Location</button>\n"
    "                        <button class=\"btn\" style=\"margin-bottom:0; width:100%; background-color:var(--accent-orange);\" onclick=\"saveNodeConfigTab()\">Save Node Config</button>\n"
    "                    </div>\n"
    "                </div>\n"
    "            </div>\n"
    "\n"
    "        </div>\n"
    "\n"
    "        <!-- Bottom Section: Methodology & Step-by-Step Guides side-by-side -->\n"
    "        <div class=\"training-grid\" style=\"margin-top: 20px;\">\n"
    "            <div class=\"training-card\" style=\"margin-bottom:0;\">\n"
    "                <h3>Training & Calibration Methodology</h3>\n"
    "                <p style=\"font-size:13px; color:var(--text-muted); line-height:1.5;\">\n"
    "                    The robot monitors its location using a time-integrated dead-reckoning kinematics model. Since surfaces have different sliding resistances and motor outputs vary by battery level, you must calibrate the travel speed and rotation rate for optimal navigation.\n"
    "                </p>\n"
    "                <div class=\"math-block\">\n"
    "                    x_t = x_t-1 + (Speed) * cos(heading) * dt<br>\n"
    "                    y_t = y_t-1 + (Speed) * sin(heading) * dt<br>\n"
    "                    heading_t = heading_t-1 + (TurnRate) * dt\n"
    "                </div>\n"
    "            </div>\n"
    "\n"
    "            <div class=\"training-card\" style=\"margin-bottom:0;\">\n"
    "                <h3>Step-by-Step Training Guide</h3>\n"
    "                \n"
    "                <div class=\"training-step\">\n"
    "                    <span class=\"training-step-num\">1</span>\n"
    "                    <div class=\"training-step-desc\">\n"
    "                        <strong>Prepare Travel Test Track:</strong> Place a physical ruler or mark exactly 1 meter (or 2 meters) forward from the robot's starting center.\n"
    "                    </div>\n"
    "                </div>\n"
    "                \n"
    "                <div class=\"training-step\">\n"
    "                    <span class=\"training-step-num\">2</span>\n"
    "                    <div class=\"training-step-desc\">\n"
    "                        <strong>Reset and Drive:</strong> Click <em>Reset Odometer</em> to align estimated coordinates to home. Go to <em>Command Center</em> and command the robot <em>Forward</em> manually until it aligns precisely with the physical 1-meter mark. Stop the robot.\n"
    "                    </div>\n"
    "                </div>\n"
    "\n"
    "                <div class=\"training-step\">\n"
    "                    <span class=\"training-step-num\">3</span>\n"
    "                    <div class=\"training-step-desc\">\n"
    "                        <strong>Calibrate Speed:</strong> Look at the <strong>Estimated X Coord</strong> above. If it reads more than the actual physical distance (e.g. it reads 1.2m but traveled 1m), <strong>decrease</strong> the <em>Travel Speed</em> value. If it reads less (e.g. 0.8m but traveled 1m), <strong>increase</strong> the <em>Travel Speed</em>.\n"
    "                    </div>\n"
    "                </div>\n"
    "\n"
    "                <div class=\"training-step\">\n"
    "                    <span class=\"training-step-num\">4</span>\n"
    "                    <div class=\"training-step-desc\">\n"
    "                        <strong>Calibrate Turning:</strong> Reset odometer. Turn the robot left/right to do exactly one full 360-degree rotation. The <strong>Estimated Heading</strong> should read exactly <code>0.0°</code> (or <code>360°</code>). If the 3D visualizer indicates it rotated more or less than the physical robot, adjust the <em>Turn Rate</em> accordingly.\n"
    "                    </div>\n"
    "                </div>\n"
    "            </div>\n"
    "        </div>\n"
    "    </div>\n"
    "\n"
    "    <!-- Tab 3: Firmware Update (OTA) -->\n"
    "    <div id=\"tab-firmware-update\" class=\"tab-content\">\n"
    "        <div style=\"max-width: 600px; margin: 40px auto;\">\n"
    "            <div class=\"panel\" style=\"border: 1px solid var(--border-color); background: var(--panel-bg); border-radius: 8px; box-shadow: 0 4px 20px rgba(0,0,0,0.4);\">\n"
    "                <div class=\"panel-header\" style=\"font-size: 16px; padding: 15px 20px; border-bottom: 1px solid var(--border-color); font-weight: 600; color: var(--accent-orange); display: flex; align-items: center; gap: 8px;\">\n"
    "                    <span>⚙️</span> Central Firmware Update (OTA)\n"
    "                </div>\n"
    "                <div class=\"panel-body\" style=\"padding: 30px 25px;\">\n"
    "                    <p style=\"font-size: 13px; color: var(--text-muted); line-height: 1.6; margin-bottom: 20px;\">\n"
    "                        Upload compiled binary files (<code style=\"color: var(--accent-orange); font-weight: 600;\">.bin</code>) directly to the robot's partition flash memory. Ensure the robot has sufficient battery level before performing updates.\n"
    "                    </p>\n"
    "                    \n"
    "                    <!-- Drag and Drop Dropzone -->\n"
    "                    <div id=\"ota-dropzone\" \n"
    "                         style=\"border: 2px dashed var(--accent-orange); border-radius: 8px; padding: 40px 20px; text-align: center; background: rgba(255, 165, 0, 0.02); cursor: pointer; transition: all 0.3s ease; margin-bottom: 25px;\"\n"
    "                         onclick=\"document.getElementById('ota_file').click()\"\n"
    "                         ondragover=\"onOtaDragOver(event)\"\n"
    "                         ondragleave=\"onOtaDragLeave(event)\"\n"
    "                         ondrop=\"onOtaDrop(event)\">\n"
    "                        <div style=\"font-size: 40px; margin-bottom: 15px; filter: drop-shadow(0 0 10px rgba(255, 165, 0, 0.3));\">📥</div>\n"
    "                        <div id=\"dropzone-text\" style=\"font-size: 14px; font-weight: 600; color: var(--text-color);\">\n"
    "                            Drag & Drop firmware (.bin) file here\n"
    "                        </div>\n"
    "                        <div style=\"font-size: 12px; color: var(--text-muted); margin-top: 5px;\">\n"
    "                            or click to browse local files\n"
    "                        </div>\n"
    "                        <input type=\"file\" id=\"ota_file\" accept=\".bin\" style=\"display:none\" onchange=\"onOtaFileSelect(this)\">\n"
    "                    </div>\n"
    "\n"
    "                    <!-- File Details Card -->\n"
    "                    <div id=\"ota-file-details\" style=\"display: none; background: var(--bg-color); border: 1px solid var(--border-color); border-radius: 6px; padding: 12px 15px; margin-bottom: 25px; align-items: center; justify-content: space-between;\">\n"
    "                        <div style=\"display: flex; align-items: center; gap: 10px;\">\n"
    "                            <span style=\"font-size: 20px;\">📄</span>\n"
    "                            <div>\n"
    "                                <div id=\"ota-file-name\" style=\"font-size: 13px; font-weight: 600; color: var(--text-color); max-width: 300px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap;\">firmware.bin</div>\n"
    "                                <div id=\"ota-file-size\" style=\"font-size: 11px; color: var(--text-muted);\">0.0 MB</div>\n"
    "                            </div>\n"
    "                        </div>\n"
    "                        <button class=\"btn\" style=\"background: transparent; border: 1px solid var(--accent-red); color: var(--accent-red); margin-bottom: 0; padding: 4px 10px; font-size: 11px;\" onclick=\"clearOtaFile(event)\">Cancel</button>\n"
    "                    </div>\n"
    "\n"
    "                    <!-- Progress Bar -->\n"
    "                    <div id=\"progress_container\" class=\"progress-container\" style=\"display: none; height: 8px; margin-bottom: 15px; background: var(--bg-color); border-radius: 4px; overflow: hidden; border: 1px solid var(--border-color);\">\n"
    "                        <div id=\"progress_bar\" class=\"progress-bar\" style=\"height: 100%; width: 0%; background: linear-gradient(90deg, var(--accent-orange), var(--accent-green)); transition: width 0.1s ease;\"></div>\n"
    "                    </div>\n"
    "                    \n"
    "                    <div id=\"ota_status\" style=\"font-size: 13px; color: var(--text-muted); text-align: center; margin-bottom: 25px; min-height: 18px;\">Ready</div>\n"
    "\n"
    "                    <button class=\"btn\" id=\"ota_btn\" style=\"width: 100%; font-size: 14px; background: var(--accent-green); padding: 12px; font-weight: 600; display: flex; align-items: center; justify-content: center; gap: 8px;\" onclick=\"uploadOTA()\">\n"
    "                        ⚡ Flash Firmware\n"
    "                    </button>\n"
    "                </div>\n"
    "            </div>\n"
    "        </div>\n"
    "    </div>\n"
    "\n"
    "    <script>\n"
    "        let isConnected = false;\n"
    "        let updateInterval = null;\n"
    "        let serverIp = \"192.168.4.1\";\n"
    "        let autoMode = true;\n"
    "        let camIp = null;\n"
    "        let hasSyncedCalibration = false;\n"
    "        let isDraggingSpeed = false;\n"
    "        let s_last_x = 0.0;\n"
    "        let s_last_y = 0.0;\n"
    "\n"
    "        function switchTab(tabId) {\n"
    "            document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));\n"
    "            document.querySelectorAll('.tab-btn').forEach(el => el.classList.remove('active'));\n"
    "            \n"
    "            if (tabId === 'command-center') {\n"
    "                document.getElementById('tab-command-center').classList.add('active');\n"
    "                document.getElementById('tab-btn-command').classList.add('active');\n"
    "            } else if (tabId === 'calibration-training') {\n"
    "                document.getElementById('tab-calibration-training').classList.add('active');\n"
    "                document.getElementById('tab-btn-calibration').classList.add('active');\n"
    "            } else if (tabId === 'firmware-update') {\n"
    "                document.getElementById('tab-firmware-update').classList.add('active');\n"
    "                document.getElementById('tab-btn-firmware').classList.add('active');\n"
    "            }\n"
    "        }\n"
    "\n"
    "        function resetOdometer() {\n"
    "            if (!isConnected) return alert(\"Please connect to the Robot first.\");\n"
    "            logMsg(\"Resetting odometer to home (0,0)...\");\n"
    "            fetch(`http://${serverIp}/cmd?go=reset_pos`, { mode: 'cors' })\n"
    "                .then(res => {\n"
    "                    logMsg(\"Odometer reset command sent.\");\n"
    "                    fetchStatus();\n"
    "                })\n"
    "                .catch(err => logMsg(\"Failed to reset odometer.\"));\n"
    "        }\n"
    "\n"
    "        function saveCalibrationTab() {\n"
    "            if (!isConnected) return alert(\"Please connect to the Robot first.\");\n"
    "            const speed = parseFloat(document.getElementById('cal-speed-input-tab').value);\n"
    "            const turn = parseFloat(document.getElementById('cal-turn-input-tab').value);\n"
    "            \n"
    "            if (isNaN(speed) || isNaN(turn)) {\n"
    "                alert(\"Please enter valid numeric values for calibration!\");\n"
    "                return;\n"
    "            }\n"
    "\n"
    "            logMsg(`Saving odometry calibration... Speed: ${speed} m/s, Turn: ${turn} rad/s`);\n"
    "            const url = `http://${serverIp}/set_calibration?speed=${speed}&turn=${turn}`;\n"
    "            fetch(url, { mode: 'cors' })\n"
    "                .then(res => {\n"
    "                    if (res.ok) {\n"
    "                        logMsg(\"Odometry calibration saved successfully!\");\n"
    "                        document.getElementById('cal-speed-input').value = speed;\n"
    "                        document.getElementById('cal-turn-input').value = turn;\n"
    "                        alert(\"Calibration saved successfully!\");\n"
    "                    } else {\n"
    "                        alert(\"Failed to save calibration to robot.\");\n"
    "                    }\n"
    "                })\n"
    "                .catch(err => {\n"
    "                    alert(\"Network error saving calibration.\");\n"
    "                    logMsg(\"<span style='color:var(--accent-red)'>Error: Network failed on calibration save.</span>\");\n"
    "                });\n"
    "        }\n"
    "\n"
    "        function onSpeedSliderInputTab(value) {\n"
    "            isDraggingSpeed = true;\n"
    "            document.getElementById('speed-val-display-tab').textContent = value + '%';\n"
    "            document.getElementById('speed-val-display').textContent = value + '%';\n"
    "            document.getElementById('motor-speed-slider').value = value;\n"
    "        }\n"
    "\n"
    "        function onSpeedSliderChangeTab(value) {\n"
    "            onSpeedSliderChange(value);\n"
    "        }\n"
    "\n"
    "        function onTrainNodeSelectChange(id) {\n"
    "            const node = nodesData[id];\n"
    "            if (!node) return;\n"
    "            document.getElementById('train-node-name').value = node.name;\n"
    "            document.getElementById('train-node-x').value = node.x.toFixed(2);\n"
    "            document.getElementById('train-node-y').value = node.y.toFixed(2);\n"
    "        }\n"
    "\n"
    "        function captureNodeLocation() {\n"
    "            document.getElementById('train-node-x').value = s_last_x.toFixed(2);\n"
    "            document.getElementById('train-node-y').value = s_last_y.toFixed(2);\n"
    "            logMsg(`Captured robot position: (${s_last_x.toFixed(2)}, ${s_last_y.toFixed(2)}) into input fields.`);\n"
    "        }\n"
    "\n"
    "        function saveNodeConfigTab() {\n"
    "            if (!isConnected) return alert(\"Please connect to the Robot first.\");\n"
    "            const id = document.getElementById('train-node-select').value;\n"
    "            const name = document.getElementById('train-node-name').value.trim();\n"
    "            const x = parseFloat(document.getElementById('train-node-x').value);\n"
    "            const y = parseFloat(document.getElementById('train-node-y').value);\n"
    "            \n"
    "            if (!name) return alert(\"Node name cannot be empty!\");\n"
    "            if (isNaN(x) || isNaN(y)) return alert(\"Invalid coordinate values.\");\n"
    "\n"
    "            logMsg(`Saving Node ${id} configuration... Name: ${name}, Position: (${x}, ${y})`);\n"
    "            const encodedName = encodeURIComponent(name);\n"
    "            const url = `http://${serverIp}/set_node?id=${id}&x=${x}&y=${y}&name=${encodedName}`;\n"
    "            \n"
    "            fetch(url, { mode: 'cors' })\n"
    "                .then(res => {\n"
    "                    if (res.ok) {\n"
    "                        logMsg(`Node ${id} config saved successfully!`);\n"
    "                        nodesData[id].name = name;\n"
    "                        nodesData[id].x = x;\n"
    "                        nodesData[id].y = y;\n"
    "                        \n"
    "                        const targetZ = -y;\n"
    "                        if (nodeMeshes[id]) {\n"
    "                            nodeMeshes[id].position.set(x, 0.12, targetZ);\n"
    "                        }\n"
    "                        if (nodeMeshes[id + \"_ring\"]) {\n"
    "                            nodeMeshes[id + \"_ring\"].position.set(x, 0.01, targetZ);\n"
    "                        }\n"
    "                        \n"
    "                        alert(`Node ${id} configuration saved successfully!`);\n"
    "                        renderNodeCard(id);\n"
    "                    } else {\n"
    "                        alert(\"Failed to save configuration to robot.\");\n"
    "                    }\n"
    "                })\n"
    "                .catch(err => {\n"
    "                    alert(\"Network error saving configuration.\");\n"
    "                    logMsg(\"<span style='color:var(--accent-red)'>Error: Network failed.</span>\");\n"
    "                });\n"
    "        }\n"
    "\n"
    "        // Local Simulation States\n"
    "        let localSimulating = false;\n"
    "        let simTargetNodeId = null;\n"
    "        let simState = 0; // 0: IDLE, 1: SCAN, 2: NAVIGATING, 6: EXTINGUISH, 7: VERIFY\n"
    "        let simRobotX = 0.0;\n"
    "        let simRobotY = 0.0;\n"
    "        let simRobotHeading = 0.0;\n"
    "        let simPump = false;\n"
    "        let simTimer = 0;\n"
    "        let waterSprayMesh = null;\n"
    "\n"
    "                // 3D Scene Variables\n"
    "        let scene, camera, renderer, robotMesh, orbitControls;\n"
    "        let pathLine, pathGeometry;\n"
    "        let pathPoints = [];\n"
    "        let nodeMeshes = {};\n"
    "        \n"
    "        // Dynamic node mapping state\n"
    "        let nodesData = {\n"
    "            1: { name: \"Node 1\", x: 1.5, y: 0.0, flame: false, gas: 0, alert: false, isEditing: false },\n"
    "            2: { name: \"Node 2\", x: 1.0, y: -1.0, flame: false, gas: 0, alert: false, isEditing: false }\n"
    "        };\n"
    "\n"
    "        // Initialize 3D View\n"
    "        function init3D() {\n"
    "            const container = document.getElementById('canvas3d');\n"
    "            const width = container.clientWidth;\n"
    "            const height = container.clientHeight;\n"
    "\n"
    "            scene = new THREE.Scene();\n"
    "            scene.background = new THREE.Color(0x05070a);\n"
    "\n"
    "            // Camera setup\n"
    "            camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);\n"
    "            camera.position.set(3, 4, 5);\n"
    "\n"
    "            // Renderer setup\n"
    "            renderer = new THREE.WebGLRenderer({ antialias: true });\n"
    "            renderer.setSize(width, height);\n"
    "            container.appendChild(renderer.domElement);\n"
    "\n"
    "            // Controls\n"
    "            orbitControls = new THREE.OrbitControls(camera, renderer.domElement);\n"
    "            orbitControls.enableDamping = true;\n"
    "            orbitControls.dampingFactor = 0.05;\n"
    "\n"
    "            // Lights\n"
    "            const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);\n"
    "            scene.add(ambientLight);\n"
    "\n"
    "            const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);\n"
    "            dirLight.position.set(5, 10, 5);\n"
    "            scene.add(dirLight);\n"
    "\n"
    "            // Floor Grid\n"
    "            const gridHelper = new THREE.GridHelper(20, 20, 0x30363d, 0x1f242c);\n"
    "            gridHelper.position.y = -0.01;\n"
    "            scene.add(gridHelper);\n"
    "\n"
    "            // Axes Helper\n"
    "            const axesHelper = new THREE.AxesHelper(1);\n"
    "            scene.add(axesHelper);\n"
    "\n"
    "            // Robot Mesh (Red Box with Wheel indicators)\n"
    "            const robotGeom = new THREE.BoxGeometry(0.3, 0.15, 0.4);\n"
    "            const robotMat = new THREE.MeshStandardMaterial({ color: 0xda3633, roughness: 0.5 });\n"
    "            robotMesh = new THREE.Mesh(robotGeom, robotMat);\n"
    "            robotMesh.position.set(0, 0.075, 0);\n"
    "            scene.add(robotMesh);\n"
    "\n"
    "            // Direction Arrow (Heading)\n"
    "            const arrowDir = new THREE.Vector3(0, 0, 1);\n"
    "            const arrowOrigin = new THREE.Vector3(0, 0.16, 0);\n"
    "            const arrowLength = 0.3;\n"
    "            const arrowColor = 0x1f6feb;\n"
    "            const arrowHelper = new THREE.ArrowHelper(arrowDir, arrowOrigin, arrowLength, arrowColor, 0.1, 0.05);\n"
    "            robotMesh.add(arrowHelper);\n"
    "\n"
    "            // Path Line\n"
    "            const maxPoints = 500;\n"
    "            pathGeometry = new THREE.BufferGeometry();\n"
    "            const positions = new Float32Array(maxPoints * 3);\n"
    "            pathGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));\n"
    "            const pathMaterial = new THREE.LineBasicMaterial({ color: 0x1f6feb, linewidth: 2 });\n"
    "            pathLine = new THREE.Line(pathGeometry, pathMaterial);\n"
    "            scene.add(pathLine);\n"
    "\n"
    "            // Add Node markers to 3D scene (scale: 1 unit = 1 meter)\n"
    "            Object.keys(nodesData).forEach(id => {\n"
    "                const pos = nodesData[id];\n"
    "                const nodeGeom = new THREE.SphereGeometry(0.12, 16, 16);\n"
    "                const nodeMat = new THREE.MeshStandardMaterial({ \n"
    "                    color: 0x238636, \n"
    "                    emissive: 0x238636, \n"
    "                    emissiveIntensity: 0.3 \n"
    "                });\n"
    "                const mesh = new THREE.Mesh(nodeGeom, nodeMat);\n"
    "                mesh.position.set(pos.x, 0.12, -pos.y);\n"
    "                scene.add(mesh);\n"
    "                nodeMeshes[id] = mesh;\n"
    "                \n"
    "                // Add ring/beacon\n"
    "                const ringGeom = new THREE.RingGeometry(0.15, 0.18, 32);\n"
    "                const ringMat = new THREE.MeshBasicMaterial({ color: 0x238636, side: THREE.DoubleSide, transparent: true, opacity: 0.6 });\n"
    "                const ringMesh = new THREE.Mesh(ringGeom, ringMat);\n"
    "                ringMesh.rotation.x = Math.PI / 2;\n"
    "                ringMesh.position.set(pos.x, 0.01, -pos.y);\n"
    "                scene.add(ringMesh);\n"
    "                nodeMeshes[id + \"_ring\"] = ringMesh;\n"
    "            });\n"
    "\n"
    "            window.addEventListener('resize', onWindowResize);\n"
    "            animate();\n"
    "            logMsg(\"3D View Initialized.\");\n"
    "            renderNodeCard(1);\n"
    "            renderNodeCard(2);\n"
    "        }\n"
    "\n"
    "        function onWindowResize() {\n"
    "            const container = document.getElementById('canvas3d');\n"
    "            camera.aspect = container.clientWidth / container.clientHeight;\n"
    "            camera.updateProjectionMatrix();\n"
    "            renderer.setSize(container.clientWidth, container.clientHeight);\n"
    "        }\n"
    "\n"
    "        let pulseTime = 0;\n"
    "        function animate() {\n"
    "            requestAnimationFrame(animate);\n"
    "            orbitControls.update();\n"
    "            \n"
    "            pulseTime += 0.05;\n"
    "            Object.keys(nodesData).forEach(id => {\n"
    "                const ring = nodeMeshes[id + \"_ring\"];\n"
    "                if (ring) {\n"
    "                    const scale = 1.0 + Math.sin(pulseTime * 2) * 0.3;\n"
    "                    ring.scale.set(scale, scale, 1);\n"
    "                    ring.material.opacity = 0.8 - (scale - 0.7) / 0.6;\n"
    "                }\n"
    "            });\n"
    "\n"
    "            // Local simulator update\n"
    "            if (localSimulating) {\n"
    "                updateLocalSimulationFrame();\n"
    "            }\n"
    "\n"
    "            renderer.render(scene, camera);\n"
    "        }\n"
    "\n"
    "        function update3DTracking(x, y, heading) {\n"
    "            robotMesh.position.set(x, 0.075, -y);\n"
    "            robotMesh.rotation.y = heading + Math.PI / 2;\n"
    "\n"
    "            const newPoint = new THREE.Vector3(x, 0.01, -y);\n"
    "            if (pathPoints.length === 0 || pathPoints[pathPoints.length - 1].distanceTo(newPoint) > 0.05) {\n"
    "                pathPoints.push(newPoint);\n"
    "                if (pathPoints.length > 500) pathPoints.shift();\n"
    "\n"
    "                const posAttribute = pathLine.geometry.attributes.position;\n"
    "                for (let i = 0; i < pathPoints.length; i++) {\n"
    "                    posAttribute.setXYZ(i, pathPoints[i].x, pathPoints[i].y, pathPoints[i].z);\n"
    "                }\n"
    "                for (let i = pathPoints.length; i < 500; i++) {\n"
    "                    posAttribute.setXYZ(i, newPoint.x, newPoint.y, newPoint.z);\n"
    "                }\n"
    "                pathLine.geometry.attributes.position.needsUpdate = true;\n"
    "            }\n"
    "\n"
    "            document.getElementById('coords-display').textContent = \n"
    "                `X: ${x.toFixed(2)}m, Y: ${y.toFixed(2)}m, H: ${(heading * 180 / Math.PI).toFixed(1)}°`;\n"
    "        }\n"
    "\n"
    "        function logMsg(msg) {\n"
    "            const box = document.getElementById('logs-box');\n"
    "            const time = new Date().toLocaleTimeString();\n"
    "            box.innerHTML += `[${time}] ${msg}<br>`;\n"
    "            box.scrollTop = box.scrollHeight;\n"
    "        }\n"
    "\n"
    "        // Local Simulation & Alert Trigger Functions\n"
    "        function triggerSimulatedAlert(id) {\n"
    "            logMsg(`Alert triggered for Node ${id}.`);\n"
    "            \n"
    "            // Set the alert state locally first\n"
    "            nodesData[id].alert = true;\n"
    "            nodesData[id].flame = true;\n"
    "            nodesData[id].gas = 600;\n"
    "            updateSensorNodeUI(id, true, 600, true);\n"
    "            \n"
    "            if (isConnected) {\n"
    "                // Online mode: Send alert HTTP POST to physical robot\n"
    "                const url = `http://${serverIp}/alert?node=${id}&flame=1&gas=600`;\n"
    "                logMsg(`Sending alert HTTP request to robot: ${url}`);\n"
    "                fetch(url, { method: 'POST', mode: 'cors' })\n"
    "                    .then(res => {\n"
    "                        if (res.ok) {\n"
    "                            logMsg(`Alert successfully sent to robot for Node ${id}!`);\n"
    "                        } else {\n"
    "                            logMsg(\"<span style='color:var(--accent-red)'>Failed to send alert to robot.</span>\");\n"
    "                        }\n"
    "                    })\n"
    "                    .catch(err => {\n"
    "                        logMsg(`<span style='color:var(--accent-red)'>Network error sending alert: ${err.message}</span>`);\n"
    "                    });\n"
    "            } else {\n"
    "                // Offline mode: Run client-side 3D simulation\n"
    "                logMsg(\"Offline mode: Running local 3D simulator demo...\");\n"
    "                startLocalSimulation(id);\n"
    "            }\n"
    "        }\n"
    "\n"
    "        function startLocalSimulation(id) {\n"
    "            if (localSimulating) {\n"
    "                logMsg(\"Simulation already in progress.\");\n"
    "                return;\n"
    "            }\n"
    "            localSimulating = true;\n"
    "            simTargetNodeId = id;\n"
    "            simState = 2; // NAVIGATING\n"
    "            simRobotX = robotMesh.position.x;\n"
    "            simRobotY = -robotMesh.position.z;\n"
    "            simRobotHeading = robotMesh.rotation.y;\n"
    "            simPump = false;\n"
    "            simTimer = 0;\n"
    "            \n"
    "            // Set robot mode & state display immediately\n"
    "            document.getElementById('robot-mode').textContent = \"AUTONOMOUS (SIM)\";\n"
    "            document.getElementById('robot-mode').className = \"telemetry-val on\";\n"
    "            document.getElementById('robot-state').textContent = \"NAVIGATING\";\n"
    "            \n"
    "            logMsg(`Simulator: Robot navigating to Node ${id} coordinates (${nodesData[id].x.toFixed(2)}m, ${nodesData[id].y.toFixed(2)}m)...`);\n"
    "        }\n"
    "\n"
    "        function createWaterSpray() {\n"
    "            if (waterSprayMesh) return;\n"
    "            const sprayGeom = new THREE.ConeGeometry(0.15, 0.8, 16);\n"
    "            const sprayMat = new THREE.MeshBasicMaterial({ \n"
    "                color: 0x1f6feb, \n"
    "                transparent: true, \n"
    "                opacity: 0.6 \n"
    "            });\n"
    "            waterSprayMesh = new THREE.Mesh(sprayGeom, sprayMat);\n"
    "            \n"
    "            // Rotate cone so its tip points forward along the robot's forward axis (+Z direction)\n"
    "            sprayGeom.rotateX(Math.PI / 2);\n"
    "            sprayGeom.translate(0, 0, 0.4); // Shift spray forward\n"
    "            \n"
    "            robotMesh.add(waterSprayMesh);\n"
    "        }\n"
    "\n"
    "        function removeWaterSpray() {\n"
    "            if (waterSprayMesh) {\n"
    "                robotMesh.remove(waterSprayMesh);\n"
    "                waterSprayMesh.geometry.dispose();\n"
    "                waterSprayMesh.material.dispose();\n"
    "                waterSprayMesh = null;\n"
    "            }\n"
    "        }\n"
    "\n"
    "        function updateLocalSimulationFrame() {\n"
    "            if (!localSimulating || !simTargetNodeId) return;\n"
    "            const targetNode = nodesData[simTargetNodeId];\n"
    "            \n"
    "            // Telemetry UI updating during simulation\n"
    "            document.getElementById('robot-mode').textContent = \"AUTONOMOUS (SIM)\";\n"
    "            document.getElementById('robot-mode').className = \"telemetry-val on\";\n"
    "            \n"
    "            const states = [\"IDLE\", \"SCAN\", \"NAVIGATING\", \"FORWARD\", \"TURN_LEFT\", \"TURN_RIGHT\", \"EXTINGUISH\", \"VERIFY\", \"ARRIVED\", \"SEARCHING\", \"RETURNING_HOME\"];\n"
    "            document.getElementById('robot-state').textContent = states[simState] || \"UNKNOWN\";\n"
    "            \n"
    "            document.getElementById('telemetry-pump').textContent = simPump ? \"ON\" : \"OFF\";\n"
    "            document.getElementById('telemetry-pump').className = `telemetry-val ${simPump ? 'on' : 'off'}`;\n"
    "            \n"
    "            // Update flame sensors text based on state\n"
    "            if (simState === 6) { // EXTINGUISH\n"
    "                document.getElementById('telemetry-flame').textContent = \"L: OK | F: FIRE | R: OK\";\n"
    "            } else if (simState === 1) { // SCAN\n"
    "                document.getElementById('telemetry-flame').textContent = \"L: OK | F: OK | R: OK\";\n"
    "            } else {\n"
    "                document.getElementById('telemetry-flame').textContent = \"L: -- | F: -- | R: --\";\n"
    "            }\n"
    "\n"
    "            if (simState === 2) { // NAVIGATING\n"
    "                const dx = targetNode.x - simRobotX;\n"
    "                const dy = targetNode.y - simRobotY;\n"
    "                const dist = Math.sqrt(dx * dx + dy * dy);\n"
    "                \n"
    "                if (dist > 0.05) {\n"
    "                    const targetHeading = Math.atan2(dy, dx);\n"
    "                    let diff = targetHeading - simRobotHeading;\n"
    "                    \n"
    "                    // Normalize heading difference to [-PI, PI]\n"
    "                    while (diff > Math.PI) diff -= 2 * Math.PI;\n"
    "                    while (diff < -Math.PI) diff += 2 * Math.PI;\n"
    "                    \n"
    "                    if (Math.abs(diff) > 0.05) {\n"
    "                        // Rotate towards heading\n"
    "                        simState = (diff > 0) ? 4 : 5; // 4: TURN_LEFT, 5: TURN_RIGHT\n"
    "                        simRobotHeading += Math.sign(diff) * 1.5 * 0.016; // 1.5 rad/s * ~16ms\n"
    "                    } else {\n"
    "                        simState = 2; // Keep in NAVIGATING\n"
    "                        // Move forward\n"
    "                        simRobotX += 0.20 * Math.cos(simRobotHeading) * 0.016; // 0.20 m/s * ~16ms\n"
    "                        simRobotY += 0.20 * Math.sin(simRobotHeading) * 0.016;\n"
    "                    }\n"
    "                    update3DTracking(simRobotX, simRobotY, simRobotHeading);\n"
    "                } else {\n"
    "                    // Arrived!\n"
    "                    simState = 1; // SCAN\n"
    "                    simTimer = 60; // 1 second scan (at 60fps)\n"
    "                    logMsg(`Simulator: Arrived at Node ${simTargetNodeId}. Beginning localized scan...`);\n"
    "                }\n"
    "            } else if (simState === 4 || simState === 5) { // Turning\n"
    "                const dx = targetNode.x - simRobotX;\n"
    "                const dy = targetNode.y - simRobotY;\n"
    "                const targetHeading = Math.atan2(dy, dx);\n"
    "                let diff = targetHeading - simRobotHeading;\n"
    "                while (diff > Math.PI) diff -= 2 * Math.PI;\n"
    "                while (diff < -Math.PI) diff += 2 * Math.PI;\n"
    "                \n"
    "                if (Math.abs(diff) > 0.05) {\n"
    "                    simRobotHeading += Math.sign(diff) * 1.5 * 0.016;\n"
    "                } else {\n"
    "                    simState = 2; // Resume navigation forward\n"
    "                }\n"
    "                update3DTracking(simRobotX, simRobotY, simRobotHeading);\n"
    "            } else if (simState === 1) { // SCAN\n"
    "                simTimer--;\n"
    "                if (simTimer <= 0) {\n"
    "                    simState = 6; // EXTINGUISH\n"
    "                    simTimer = 180; // 3 seconds of extinguishing (at 60fps)\n"
    "                    simPump = true;\n"
    "                    createWaterSpray();\n"
    "                    logMsg(\"Simulator: Fire detected! Water pump ON, sweeping nozzle...\");\n"
    "                }\n"
    "            } else if (simState === 6) { // EXTINGUISH\n"
    "                simTimer--;\n"
    "                if (waterSprayMesh) {\n"
    "                    waterSprayMesh.rotation.y = Math.sin(pulseTime * 5) * 0.5; // Nozzle sweep animation\n"
    "                }\n"
    "                if (simTimer <= 0) {\n"
    "                    simState = 7; // VERIFY\n"
    "                    simTimer = 60; // 1 second verification\n"
    "                    simPump = false;\n"
    "                    removeWaterSpray();\n"
    "                    logMsg(\"Simulator: Sweep complete. Turning pump OFF, verifying...\");\n"
    "                }\n"
    "            } else if (simState === 7) { // VERIFY\n"
    "                simTimer--;\n"
    "                if (simTimer <= 0) {\n"
    "                    simState = 0; // IDLE\n"
    "                    localSimulating = false;\n"
    "                    \n"
    "                    // Reset alert states\n"
    "                    nodesData[simTargetNodeId].alert = false;\n"
    "                    nodesData[simTargetNodeId].flame = false;\n"
    "                    nodesData[simTargetNodeId].gas = 80; // normal\n"
    "                    updateSensorNodeUI(simTargetNodeId, false, 80, false);\n"
    "                    \n"
    "                    logMsg(\"Simulator: Fire extinguished. Returning to standby.\");\n"
    "                }\n"
    "            }\n"
    "        }\n"
    "\n"
    "        let pollTimeout = null;\n"
    "\n"
    "        function disconnect() {\n"
    "            if (pollTimeout) {\n"
    "                clearTimeout(pollTimeout);\n"
    "                pollTimeout = null;\n"
    "            }\n"
    "            isConnected = false;\n"
    "            hasSyncedCalibration = false;\n"
    "            document.getElementById('conn-dot').className = \"status-dot\";\n"
    "            const btn = document.getElementById('connect-btn');\n"
    "            btn.textContent = \"Connect\";\n"
    "            btn.style.backgroundColor = \"var(--accent-blue)\";\n"
    "            logMsg(\"Disconnected from Robot server.\");\n"
    "            document.getElementById('stream-img').style.display = 'none';\n"
    "            document.getElementById('video-placeholder').style.display = 'flex';\n"
    "        }\n"
    "\n"
    "        function toggleConnect() {\n"
    "            serverIp = document.getElementById('robot-ip').value;\n"
    "            const btn = document.getElementById('connect-btn');\n"
    "            \n"
    "            if (isConnected) {\n"
    "                disconnect();\n"
    "            } else {\n"
    "                logMsg(`Attempting to connect to http://${serverIp}/...`);\n"
    "                btn.textContent = \"Connecting...\";\n"
    "                \n"
    "                fetch(`http://${serverIp}/status`, { mode: 'cors' })\n"
    "                    .then(res => res.json())\n"
    "                    .then(data => {\n"
    "                        isConnected = true;\n"
    "                        document.getElementById('conn-dot').className = \"status-dot connected\";\n"
    "                        btn.textContent = \"Disconnect\";\n"
    "                        btn.style.backgroundColor = \"var(--accent-red)\";\n"
    "                        logMsg(\"Connected successfully!\");\n"
    "                        \n"
    "                        // Parse first status payload and update UI\n"
    "                        updateUI(data);\n"
    "                        \n"
    "                        // Start polling with recursive setTimeout (avoids simultaneous double-fetch)\n"
    "                        pollTimeout = setTimeout(fetchStatus, 300);\n"
    "                    })\n"
    "                    .catch(err => {\n"
    "                        logMsg(\"<span style='color:var(--accent-red)'>Error: Connection failed. Check Wi-Fi connection and IP address.</span>\");\n"
    "                        btn.textContent = \"Connect\";\n"
    "                    });\n"
    "            }\n"
    "        }\n"
    "\n"
    "        function fetchStatus() {\n"
    "            if (!isConnected) return;\n"
    "            \n"
    "            fetch(`http://${serverIp}/status`, { mode: 'cors' })\n"
    "                .then(res => res.json())\n"
    "                .then(data => {\n"
    "                    if (!isConnected) return;\n"
    "                    updateUI(data);\n"
    "                    \n"
    "                    // Schedule next poll only AFTER this fetch completes successfully\n"
    "                    pollTimeout = setTimeout(fetchStatus, 300);\n"
    "                })\n"
    "                .catch(err => {\n"
    "                    if (!isConnected) return;\n"
    "                    logMsg(\"<span style='color:var(--accent-red)'>Connection interrupted.</span>\");\n"
    "                    disconnect();\n"
    "                });\n"
    "        }\n"
    "\n"
    "        function updateUI(data) {\n"
    "            autoMode = data.auto_mode;\n"
    "            const modeBtn = document.getElementById('mode-btn');\n"
    "            modeBtn.textContent = autoMode ? \"Switch to Manual\" : \"Switch to Auto\";\n"
    "            modeBtn.style.backgroundColor = autoMode ? \"var(--accent-orange)\" : \"var(--accent-green)\";\n"
    "            \n"
    "            const modeBtnTab = document.getElementById('mode-btn-tab');\n"
    "            if (modeBtnTab) {\n"
    "                modeBtnTab.textContent = autoMode ? \"Switch to Manual\" : \"Switch to Auto\";\n"
    "                modeBtnTab.style.backgroundColor = autoMode ? \"var(--accent-orange)\" : \"var(--accent-green)\";\n"
    "            }\n"
    "\n"
    "            document.getElementById('robot-mode').textContent = autoMode ? \"AUTONOMOUS\" : \"MANUAL\";\n"
    "            document.getElementById('robot-mode').className = `telemetry-val ${autoMode ? 'on' : 'off'}`;\n"
    "            \n"
    "            const states = [\"IDLE\", \"SCAN\", \"NAVIGATING\", \"FORWARD\", \"TURN_LEFT\", \"TURN_RIGHT\", \"EXTINGUISH\", \"VERIFY\", \"ARRIVED\", \"SEARCHING\", \"RETURNING_HOME\"];\n"
    "            document.getElementById('robot-state').textContent = states[data.state] || \"UNKNOWN\";\n"
    "\n"
    "            document.getElementById('telemetry-flame').textContent = \n"
    "                `L: ${data.flame_l?'FIRE':'OK'} | F: ${data.flame_f?'FIRE':'OK'} | R: ${data.flame_r?'FIRE':'OK'}`;\n"
    "            \n"
    "            const pumpOn = data.pump;\n"
    "            document.getElementById('telemetry-pump').textContent = pumpOn ? \"ON\" : \"OFF\";\n"
    "            document.getElementById('telemetry-pump').className = `telemetry-val ${pumpOn ? 'on' : 'off'}`;\n"
    "\n"
    "            s_last_x = data.x;\n"
    "            s_last_y = data.y;\n"
    "            update3DTracking(data.x, data.y, data.heading);\n"
    "\n"
    "            // Update real-time status in training tab\n"
    "            document.getElementById('train-x-val').textContent = data.x.toFixed(2) + 'm';\n"
    "            document.getElementById('train-y-val').textContent = data.y.toFixed(2) + 'm';\n"
    "            document.getElementById('train-h-val').textContent = (data.heading * 180 / Math.PI).toFixed(1) + '°';\n"
    "\n"
    "            if (!isDraggingSpeed && data.motor_speed !== undefined) {\n"
    "                document.getElementById('motor-speed-slider').value = data.motor_speed;\n"
    "                document.getElementById('speed-val-display').textContent = data.motor_speed + '%';\n"
    "                const sliderTab = document.getElementById('motor-speed-slider-tab');\n"
    "                if (sliderTab) {\n"
    "                    sliderTab.value = data.motor_speed;\n"
    "                    document.getElementById('speed-val-display-tab').textContent = data.motor_speed + '%';\n"
    "                }\n"
    "            }\n"
    "\n"
    "            if (!hasSyncedCalibration && data.cal_speed !== undefined && data.cal_turn !== undefined) {\n"
    "                document.getElementById('cal-speed-input').value = data.cal_speed;\n"
    "                document.getElementById('cal-turn-input').value = data.cal_turn;\n"
    "                document.getElementById('cal-speed-input-tab').value = data.cal_speed;\n"
    "                document.getElementById('cal-turn-input-tab').value = data.cal_turn;\n"
    "                hasSyncedCalibration = true;\n"
    "                logMsg(`Synced calibration parameters from robot: Speed = ${data.cal_speed} m/s, Turn = ${data.cal_turn} rad/s`);\n"
    "                \n"
    "                // Initialize Training tab node config fields\n"
    "                onTrainNodeSelectChange(\"1\");\n"
    "            }\n"
    "\n"
    "            updateSensorNodeUI(1, data.node1_flame, data.node1_gas, data.node1_alert, data.node1_x, data.node1_y, data.node1_name);\n"
    "            updateSensorNodeUI(2, data.node2_flame, data.node2_gas, data.node2_alert, data.node2_x, data.node2_y, data.node2_name);\n"
    "\n"
    "            if (data.cam_ip && data.cam_ip !== camIp) {\n"
    "                camIp = data.cam_ip;\n"
    "                const streamImg = document.getElementById('stream-img');\n"
    "                streamImg.src = `http://${camIp}:81/stream`;\n"
    "                streamImg.style.display = 'block';\n"
    "                document.getElementById('video-placeholder').style.display = 'none';\n"
    "                logMsg(`Camera detected at IP ${camIp}. Loading stream...`);\n"
    "            } else if (!data.cam_ip && camIp) {\n"
    "                camIp = null;\n"
    "                document.getElementById('stream-img').style.display = 'none';\n"
    "                document.getElementById('video-placeholder').style.display = 'flex';\n"
    "                logMsg(\"Camera disconnected.\");\n"
    "            }\n"
    "        }\n"
    "\n"
    "        function updateSensorNodeUI(nodeId, hasFlame, gasLevel, activeAlert, x, y, name) {\n"
    "            const node = nodesData[nodeId];\n"
    "            if (!node) return;\n"
    "            \n"
    "            if (node.isEditing) {\n"
    "                node.flame = hasFlame;\n"
    "                node.gas = gasLevel;\n"
    "                node.alert = activeAlert;\n"
    "                return;\n"
    "            }\n"
    "            \n"
    "            node.flame = hasFlame;\n"
    "            node.gas = gasLevel;\n"
    "            node.alert = activeAlert;\n"
    "            \n"
    "            if (x !== undefined && y !== undefined) {\n"
    "                node.x = x;\n"
    "                node.y = y;\n"
    "                const targetZ = -y;\n"
    "                if (nodeMeshes[nodeId]) {\n"
    "                    nodeMeshes[nodeId].position.set(x, 0.12, targetZ);\n"
    "                }\n"
    "                if (nodeMeshes[nodeId + \"_ring\"]) {\n"
    "                    nodeMeshes[nodeId + \"_ring\"].position.set(x, 0.01, targetZ);\n"
    "                }\n"
    "            }\n"
    "            \n"
    "            if (name !== undefined && name !== \"\") {\n"
    "                node.name = name;\n"
    "            }\n"
    "\n"
    "            // Update Three.js mesh color according to alert state\n"
    "            if (nodeMeshes[nodeId]) {\n"
    "                nodeMeshes[nodeId].material.color.setHex(activeAlert ? 0xda3633 : 0x238636);\n"
    "                nodeMeshes[nodeId].material.emissive.setHex(activeAlert ? 0xda3633 : 0x238636);\n"
    "            }\n"
    "            if (nodeMeshes[nodeId + \"_ring\"]) {\n"
    "                nodeMeshes[nodeId + \"_ring\"].material.color.setHex(activeAlert ? 0xda3633 : 0x238636);\n"
    "            }\n"
    "            \n"
    "            renderNodeCard(nodeId);\n"
    "        }\n"
    "\n"
    "        function renderNodeCard(id) {\n"
    "            const card = document.getElementById(`node-card-${id}`);\n"
    "            if (!card) return;\n"
    "            const node = nodesData[id];\n"
    "            \n"
    "            if (node.isEditing) {\n"
    "                card.innerHTML = `\n"
    "                    <div class=\"node-edit-form\" style=\"font-family:inherit;\">\n"
    "                        <div style=\"margin-bottom:8px;\">\n"
    "                            <label style=\"font-size:11px;color:var(--text-muted);display:block;\">Node Name:</label>\n"
    "                            <input type=\"text\" id=\"edit-name-${id}\" value=\"${node.name}\" style=\"width:100%;padding:4px 8px;background:var(--bg-color);border:1px solid var(--border-color);color:var(--text-color);border-radius:4px;margin-top:2px;box-sizing:border-box;\">\n"
    "                        </div>\n"
    "                        <div style=\"display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-bottom:8px;\">\n"
    "                            <div>\n"
    "                                <label style=\"font-size:11px;color:var(--text-muted);display:block;\">X Position (m):</label>\n"
    "                                <input type=\"number\" step=\"0.1\" id=\"edit-x-${id}\" value=\"${node.x}\" style=\"width:100%;padding:4px 8px;background:var(--bg-color);border:1px solid var(--border-color);color:var(--text-color);border-radius:4px;margin-top:2px;box-sizing:border-box;\">\n"
    "                            </div>\n"
    "                            <div>\n"
    "                                <label style=\"font-size:11px;color:var(--text-muted);display:block;\">Y Position (m):</label>\n"
    "                                <input type=\"number\" step=\"0.1\" id=\"edit-y-${id}\" value=\"${node.y}\" style=\"width:100%;padding:4px 8px;background:var(--bg-color);border:1px solid var(--border-color);color:var(--text-color);border-radius:4px;margin-top:2px;box-sizing:border-box;\">\n"
    "                            </div>\n"
    "                        </div>\n"
    "                        <div style=\"display:flex;gap:6px;\">\n"
    "                            <button class=\"btn\" style=\"padding:4px 8px;font-size:11px;background-color:var(--accent-green);margin-bottom:0;\" onclick=\"saveNodeEdit(${id})\">Save</button>\n"
    "                            <button class=\"btn\" style=\"padding:4px 8px;font-size:11px;background-color:var(--border-color);color:var(--text-color);margin-bottom:0;\" onclick=\"cancelNodeEdit(${id})\">Cancel</button>\n"
    "                        </div>\n"
    "                    </div>\n"
    "                `;\n"
    "            } else {\n"
    "                const isAlert = node.alert;\n"
    "                if (isAlert) {\n"
    "                    card.className = \"node-card alert\";\n"
    "                } else {\n"
    "                    card.className = \"node-card\";\n"
    "                }\n"
    "                \n"
    "                const statusText = isAlert ? (node.flame ? \"FIRE ALARM\" : \"GAS ALARM\") : \"OK\";\n"
    "                const fillPercent = Math.min(100, Math.max(0, (node.gas / 1023) * 100));\n"
    "                const barColor = node.gas > 400 ? \"var(--accent-red)\" : (node.gas > 200 ? \"var(--accent-orange)\" : \"#39d353\");\n"
    "                \n"
    "                card.innerHTML = `\n"
    "                    <div class=\"node-card-header\" style=\"display:flex;justify-content:space-between;align-items:center;margin-bottom:6px;\">\n"
    "                        <span style=\"display:flex;align-items:center;gap:6px;font-weight:600;font-size:13px;\">\n"
    "                            ${node.name}\n"
    "                            <span style=\"cursor:pointer;font-size:12px;opacity:0.6;\" onclick=\"startNodeEdit(${id})\">✎</span>\n"
    "                        </span>\n"
    "                        <span class=\"node-status\" style=\"font-size:11px;padding:2px 5px;border-radius:8px;background:${isAlert ? 'var(--accent-red)' : 'var(--border-color)'};color:white;\">${statusText}</span>\n"
    "                    </div>\n"
    "                    <div style=\"font-size:12px;display:flex;justify-content:space-between;\">\n"
    "                        <span>MQ2 Gas Level:</span>\n"
    "                        <span>${node.gas} ppm</span>\n"
    "                    </div>\n"
    "                    <div class=\"gas-meter\" style=\"height:6px;background:#0d1117;border-radius:3px;overflow:hidden;margin-top:4px;\">\n"
    "                        <div class=\"gas-fill\" style=\"height:100%;width:${fillPercent}%;background-color:${barColor};transition:width 0.3s;\"></div>\n"
    "                    </div>\n"
    "                    <div style=\"font-size:12px;display:flex;justify-content:space-between;margin-top:6px;\">\n"
    "                        <span>Flame Sensor:</span>\n"
    "                        <span style=\"font-weight:600;color:${node.flame ? 'var(--accent-red)' : '#39d353'}\">${node.flame ? 'FLAME DETECTED' : 'OK'}</span>\n"
    "                    </div>\n"
    "                    <div style=\"font-size:10px;color:var(--text-muted);margin-top:6px;display:flex;justify-content:space-between;align-items:center;\">\n"
    "                        <span>Pos: (${node.x.toFixed(2)}m, ${node.y.toFixed(2)}m)</span>\n"
    "                        <button class=\"btn\" style=\"padding:2px 6px;font-size:10px;background-color:var(--accent-orange);margin:0;\" onclick=\"triggerSimulatedAlert(${id})\">Simulate Alert</button>\n"
    "                    </div>\n"
    "                `;\n"
    "            }\n"
    "        }\n"
    "\n"
    "        function startNodeEdit(id) {\n"
    "            nodesData[id].isEditing = true;\n"
    "            renderNodeCard(id);\n"
    "        }\n"
    "        \n"
    "        function cancelNodeEdit(id) {\n"
    "            nodesData[id].isEditing = false;\n"
    "            renderNodeCard(id);\n"
    "        }\n"
    "        \n"
    "        function saveNodeEdit(id) {\n"
    "            const name = document.getElementById(`edit-name-${id}`).value.trim();\n"
    "            const x = parseFloat(document.getElementById(`edit-x-${id}`).value);\n"
    "            const y = parseFloat(document.getElementById(`edit-y-${id}`).value);\n"
    "            \n"
    "            if (!name) {\n"
    "                alert(\"Node name cannot be empty!\");\n"
    "                return;\n"
    "            }\n"
    "            if (isNaN(x) || isNaN(y)) {\n"
    "                alert(\"Please enter valid coordinates!\");\n"
    "                return;\n"
    "            }\n"
    "            \n"
    "            logMsg(`Saving Node ${id} config...`);\n"
    "            \n"
    "            const encodedName = encodeURIComponent(name);\n"
    "            const url = `http://${serverIp}/set_node?id=${id}&x=${x}&y=${y}&name=${encodedName}`;\n"
    "            \n"
    "            fetch(url, { mode: 'cors' })\n"
    "                .then(res => {\n"
    "                    if (res.ok) {\n"
    "                        logMsg(`Node ${id} saved successfully!`);\n"
    "                        nodesData[id].name = name;\n"
    "                        nodesData[id].x = x;\n"
    "                        nodesData[id].y = y;\n"
    "                        nodesData[id].isEditing = false;\n"
    "                        \n"
    "                        const targetZ = -y;\n"
    "                        if (nodeMeshes[id]) {\n"
    "                            nodeMeshes[id].position.set(x, 0.12, targetZ);\n"
    "                        }\n"
    "                        if (nodeMeshes[id + \"_ring\"]) {\n"
    "                            nodeMeshes[id + \"_ring\"].position.set(x, 0.01, targetZ);\n"
    "                        }\n"
    "                        \n"
    "                        renderNodeCard(id);\n"
    "                    } else {\n"
    "                        alert(\"Failed to save configuration to robot.\");\n"
    "                        logMsg(\"<span style='color:var(--accent-red)'>Error saving node config.</span>\");\n"
    "                    }\n"
    "                })\n"
    "                .catch(err => {\n"
    "                    alert(\"Network error. Could not connect to robot.\");\n"
    "                    logMsg(\"<span style='color:var(--accent-red)'>Error: Network connection failed.</span>\");\n"
    "                });\n"
    "        }\n"
    "\n"
    "        function toggleMode() {\n"
    "            if (!isConnected) return alert(\"Please connect to the Robot first.\");\n"
    "            const target = autoMode ? \"manual\" : \"auto\";\n"
    "            sendCmd(target);\n"
    "        }\n"
    "\n"
    "        function sendCmd(cmd) {\n"
    "            if (!isConnected) return;\n"
    "            logMsg(`Sending command: ${cmd}`);\n"
    "            fetch(`http://${serverIp}/cmd?go=${cmd}`, { mode: 'cors' })\n"
    "                .then(res => fetchStatus())\n"
    "                .catch(err => logMsg(`Failed to send command ${cmd}`));\n"
    "        }\n"
    "\n"
    "        function onSpeedSliderInput(value) {\n"
    "            isDraggingSpeed = true;\n"
    "            document.getElementById('speed-val-display').textContent = value + '%';\n"
    "        }\n"
    "\n"
    "        function onSpeedSliderChange(value) {\n"
    "            isDraggingSpeed = false;\n"
    "            if (!isConnected) return;\n"
    "            logMsg(`Adjusting motor speed limit: ${value}%`);\n"
    "            fetch(`http://${serverIp}/cmd?go=speed&val=${value}`, { mode: 'cors' })\n"
    "                .then(res => fetchStatus())\n"
    "                .catch(err => logMsg(`Failed to set speed limit`));\n"
    "        }\n"
    "\n"
    "        function saveCalibration() {\n"
    "            if (!isConnected) return alert(\"Please connect to the Robot first.\");\n"
    "            const speed = parseFloat(document.getElementById('cal-speed-input').value);\n"
    "            const turn = parseFloat(document.getElementById('cal-turn-input').value);\n"
    "            \n"
    "            if (isNaN(speed) || isNaN(turn)) {\n"
    "                alert(\"Please enter valid numeric values for calibration!\");\n"
    "                return;\n"
    "            }\n"
    "\n"
    "            logMsg(`Saving odometry calibration... Speed: ${speed} m/s, Turn: ${turn} rad/s`);\n"
    "            const url = `http://${serverIp}/set_calibration?speed=${speed}&turn=${turn}`;\n"
    "            fetch(url, { mode: 'cors' })\n"
    "                .then(res => {\n"
    "                    if (res.ok) {\n"
    "                        logMsg(\"Odometry calibration saved successfully!\");\n"
    "                        document.getElementById('cal-speed-input-tab').value = speed;\n"
    "                        document.getElementById('cal-turn-input-tab').value = turn;\n"
    "                        alert(\"Calibration saved successfully!\");\n"
    "                    } else {\n"
    "                        alert(\"Failed to save calibration to robot.\");\n"
    "                    }\n"
    "                })\n"
    "                .catch(err => {\n"
    "                    alert(\"Network error saving calibration.\");\n"
    "                    logMsg(\"<span style='color:var(--accent-red)'>Error: Network failed on calibration save.</span>\");\n"
    "                });\n"
    "        }\n"
    "\n"
    "        let otaSelectedFile = null;\n"
    "\n"
    "        function onOtaDragOver(e) {\n"
    "            e.preventDefault();\n"
    "            const dropzone = document.getElementById('ota-dropzone');\n"
    "            dropzone.style.borderColor = 'var(--accent-green)';\n"
    "            dropzone.style.background = 'rgba(57, 211, 83, 0.05)';\n"
    "        }\n"
    "\n"
    "        function onOtaDragLeave(e) {\n"
    "            e.preventDefault();\n"
    "            const dropzone = document.getElementById('ota-dropzone');\n"
    "            dropzone.style.borderColor = 'var(--accent-orange)';\n"
    "            dropzone.style.background = 'rgba(255, 165, 0, 0.02)';\n"
    "        }\n"
    "\n"
    "        function onOtaDrop(e) {\n"
    "            e.preventDefault();\n"
    "            onOtaDragLeave(e);\n"
    "            if (e.dataTransfer.files.length > 0) {\n"
    "                const file = e.dataTransfer.files[0];\n"
    "                if (file.name.endsWith('.bin')) {\n"
    "                    document.getElementById('ota_file').files = e.dataTransfer.files;\n"
    "                    handleOtaFileSelection(file);\n"
    "                } else {\n"
    "                    alert(\"Only .bin firmware files are allowed!\");\n"
    "                }\n"
    "            }\n"
    "        }\n"
    "\n"
    "        function onOtaFileSelect(input) {\n"
    "            if (input.files.length > 0) {\n"
    "                handleOtaFileSelection(input.files[0]);\n"
    "            }\n"
    "        }\n"
    "\n"
    "        function handleOtaFileSelection(file) {\n"
    "            otaSelectedFile = file;\n"
    "            document.getElementById('ota-file-name').textContent = file.name;\n"
    "            document.getElementById('ota-file-size').textContent = (file.size / (1024 * 1024)).toFixed(2) + \" MB\";\n"
    "            document.getElementById('ota-file-details').style.display = 'flex';\n"
    "            document.getElementById('ota-dropzone').style.display = 'none';\n"
    "            document.getElementById('ota_status').textContent = 'File selected. Click Flash Firmware to begin.';\n"
    "        }\n"
    "\n"
    "        function clearOtaFile(e) {\n"
    "            if (e) e.stopPropagation();\n"
    "            otaSelectedFile = null;\n"
    "            document.getElementById('ota_file').value = \"\";\n"
    "            document.getElementById('ota-file-details').style.display = 'none';\n"
    "            document.getElementById('ota-dropzone').style.display = 'block';\n"
    "            document.getElementById('ota_status').textContent = 'Ready';\n"
    "            document.getElementById('progress_container').style.display = 'none';\n"
    "            document.getElementById('progress_bar').style.width = '0%';\n"
    "            document.getElementById('ota_btn').disabled = false;\n"
    "        }\n"
    "\n"
    "        function uploadOTA() {\n"
    "            let file = otaSelectedFile || document.getElementById('ota_file').files[0];\n"
    "            if (!file) {\n"
    "                alert(\"Please select or drag a firmware file first!\");\n"
    "                return;\n"
    "            }\n"
    "            let btn = document.getElementById('ota_btn');\n"
    "            let p_container = document.getElementById('progress_container');\n"
    "            let p_bar = document.getElementById('progress_bar');\n"
    "            let status = document.getElementById('ota_status');\n"
    "            \n"
    "            btn.disabled = true;\n"
    "            p_container.style.display = 'block';\n"
    "            p_bar.style.width = '0%';\n"
    "            p_bar.style.backgroundColor = 'var(--accent-orange)';\n"
    "            status.textContent = 'Uploading: ' + file.name;\n"
    "            \n"
    "            let xhr = new XMLHttpRequest();\n"
    "            xhr.open('POST', `http://${serverIp}/ota/update`, true);\n"
    "            \n"
    "            xhr.onload = function () {\n"
    "                if (xhr.status === 200 && xhr.responseText === 'success') {\n"
    "                    p_bar.style.backgroundColor = '#39d353';\n"
    "                    status.innerHTML = \"<span style='color:#39d353'>Update Successful! Robot is rebooting...</span>\";\n"
    "                    logMsg(\"OTA Update successful! Connection will close.\");\n"
    "                    alert('Firmware updated successfully! The robot will now reboot.');\n"
    "                    clearOtaFile();\n"
    "                } else {\n"
    "                    p_bar.style.backgroundColor = 'var(--accent-red)';\n"
    "                    status.innerHTML = \"<span style='color:var(--accent-red)'>Failed: \" + xhr.responseText + \"</span>\";\n"
    "                    btn.disabled = false;\n"
    "                }\n"
    "            };\n"
    "            \n"
    "            xhr.onerror = function () {\n"
    "                p_bar.style.backgroundColor = 'var(--accent-red)';\n"
    "                status.innerHTML = \"<span style='color:var(--accent-red)'>Connection Error during upload</span>\";\n"
    "                btn.disabled = false;\n"
    "            };\n"
    "            \n"
    "            xhr.upload.onprogress = function (e) {\n"
    "                if (e.lengthComputable) {\n"
    "                    let percent = Math.round((e.loaded / e.total) * 100);\n"
    "                    p_bar.style.width = percent + '%';\n"
    "                    status.textContent = 'Uploading (' + percent + '%)';\n"
    "                }\n"
    "            };\n"
    "            \n"
    "            xhr.send(file);\n"
    "        }\n"
    "\n"
    "        window.onload = function() {\n"
    "            init3D();\n"
    "        };\n"
    "    </script>\n"
    "</body>\n"
    "</html>\n"
;

static void send_command(wifi_server_command_t command)
{
    if (s_command_cb != NULL) {
        s_command_cb(command);
    }
}

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t cmd_handler(httpd_req_t *req)
{
    char query[64] = {0};
    char value[24] = {0};

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "go", value, sizeof(value)) == ESP_OK) {
        if (strcmp(value, "auto") == 0) {
            send_command(WIFI_CMD_AUTO);
        } else if (strcmp(value, "manual") == 0) {
            send_command(WIFI_CMD_MANUAL);
        } else if (strcmp(value, "forward") == 0) {
            send_command(WIFI_CMD_FORWARD);
        } else if (strcmp(value, "reverse") == 0) {
            send_command(WIFI_CMD_REVERSE);
        } else if (strcmp(value, "left") == 0) {
            send_command(WIFI_CMD_LEFT);
        } else if (strcmp(value, "right") == 0) {
            send_command(WIFI_CMD_RIGHT);
        } else if (strcmp(value, "stop") == 0) {
            send_command(WIFI_CMD_STOP);
        } else if (strcmp(value, "pump_on") == 0) {
            send_command(WIFI_CMD_PUMP_ON);
        } else if (strcmp(value, "pump_off") == 0) {
            send_command(WIFI_CMD_PUMP_OFF);
        } else if (strcmp(value, "relay_low") == 0) {
            send_command(WIFI_CMD_RELAY_LOW);
        } else if (strcmp(value, "relay_high") == 0) {
            send_command(WIFI_CMD_RELAY_HIGH);
        } else if (strcmp(value, "sweep") == 0) {
            send_command(WIFI_CMD_SWEEP);
        } else if (strcmp(value, "reset_pos") == 0) {
            extern void reset_robot_position(void);
            reset_robot_position();
        } else if (strcmp(value, "speed") == 0) {
            char val_str[16] = {0};
            if (httpd_query_key_value(query, "val", val_str, sizeof(val_str)) == ESP_OK) {
                int speed_val = atoi(val_str);
                extern void motor_set_base_speed(int percent);
                motor_set_base_speed(speed_val);
            }
        }
    }

    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t status_handler(httpd_req_t *req)
{
    float x, y, heading;
    get_robot_position(&x, &y, &heading);

    bool node1_flame, node1_alert;
    int node1_gas;
    get_node_status(1, &node1_flame, &node1_gas, &node1_alert);

    bool node2_flame, node2_alert;
    int node2_gas;
    get_node_status(2, &node2_flame, &node2_gas, &node2_alert);

    float n1_x, n1_y, n2_x, n2_y;
    char n1_name[32], n2_name[32];
    get_node_config(1, &n1_x, &n1_y, n1_name, sizeof(n1_name));
    get_node_config(2, &n2_x, &n2_y, n2_name, sizeof(n2_name));

    float cal_speed = 0.20f;
    float cal_turn = 1.50f;
    extern void get_calibration_values(float *speed, float *turn);
    get_calibration_values(&cal_speed, &cal_turn);

    extern int motor_get_base_speed(void);
    int motor_speed = motor_get_base_speed();

    char response[1024];
    snprintf(response, sizeof(response),
             "{\"flame_l\":%d,\"flame_f\":%d,\"flame_r\":%d,\"pump\":%d,"
             "\"auto_mode\":%d,\"state\":%d,\"x\":%.3f,\"y\":%.3f,\"heading\":%.3f,"
             "\"cam_ip\":\"%s\",\"cal_speed\":%.3f,\"cal_turn\":%.3f,\"motor_speed\":%d,"
             "\"node1_flame\":%d,\"node1_gas\":%d,\"node1_alert\":%d,"
             "\"node1_x\":%.3f,\"node1_y\":%.3f,\"node1_name\":\"%s\","
             "\"node2_flame\":%d,\"node2_gas\":%d,\"node2_alert\":%d,"
             "\"node2_x\":%.3f,\"node2_y\":%.3f,\"node2_name\":\"%s\"}",
             flame_left(), flame_front(), flame_right(), relay_is_on(),
             get_auto_mode(), get_robot_state(), x, y, heading,
             s_cam_ip, cal_speed, cal_turn, motor_speed,
             node1_flame, node1_gas, node1_alert,
             n1_x, n1_y, n1_name,
             node2_flame, node2_gas, node2_alert,
             n2_x, n2_y, n2_name);

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, response);
}

static esp_err_t set_node_handler(httpd_req_t *req)
{
    char query[256] = {0};
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        int id = -1;
        float x = 0.0f;
        float y = 0.0f;
        char name[64] = "";

        char *token = strtok(query, "&");
        while (token != NULL) {
            char *eq = strchr(token, '=');
            if (eq) {
                *eq = '\0';
                char *key = token;
                char *val = eq + 1;
                if (strcmp(key, "id") == 0) {
                    id = atoi(val);
                } else if (strcmp(key, "x") == 0) {
                    x = atof(val);
                } else if (strcmp(key, "y") == 0) {
                    y = atof(val);
                } else if (strcmp(key, "name") == 0) {
                    url_decode(name, val);
                }
            }
            token = strtok(NULL, "&");
        }

        if (id >= 1 && id <= 2) {
            update_node_config(id, x, y, name);
            return httpd_resp_sendstr(req, "ok");
        }
    }

    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid parameters");
}

static esp_err_t set_calibration_handler(httpd_req_t *req)
{
    char query[128] = {0};
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        float speed = 0.20f;
        float turn = 1.50f;
        bool has_speed = false;
        bool has_turn = false;

        char *token = strtok(query, "&");
        while (token != NULL) {
            char *eq = strchr(token, '=');
            if (eq) {
                *eq = '\0';
                char *key = token;
                char *val = eq + 1;
                if (strcmp(key, "speed") == 0) {
                    speed = atof(val);
                    has_speed = true;
                } else if (strcmp(key, "turn") == 0) {
                    turn = atof(val);
                    has_turn = true;
                }
            }
            token = strtok(NULL, "&");
        }

        if (has_speed && has_turn) {
            extern void set_calibration_values(float speed, float turn);
            set_calibration_values(speed, turn);
            return httpd_resp_sendstr(req, "ok");
        }
    }

    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid parameters");
}

static esp_err_t cam_ping_handler(httpd_req_t *req)
{
    char query[64] = {0};
    char ip_val[32] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "ip", ip_val, sizeof(ip_val)) == ESP_OK) {
        strncpy(s_cam_ip, ip_val, sizeof(s_cam_ip) - 1);
        ESP_LOGI(TAG, "ESP32-CAM Ping: IP registered as %s", s_cam_ip);
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t alert_handler(httpd_req_t *req)
{
    char query[128] = {0};
    char node_val[16] = {0};
    char flame_val[16] = {0};
    char gas_val[16] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "node", node_val, sizeof(node_val));
        httpd_query_key_value(query, "flame", flame_val, sizeof(flame_val));
        httpd_query_key_value(query, "gas", gas_val, sizeof(gas_val));

        int node_id = atoi(node_val);
        bool has_flame = atoi(flame_val) == 1;
        int gas_level = atoi(gas_val);

        set_node_alert(node_id, has_flame, gas_level);
        ESP_LOGI(TAG, "Alert details - Node: %d, Flame: %d, Gas: %d", node_id, has_flame, gas_level);
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t options_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, GET, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t send_ota_error(httpd_req_t *req, const char *msg)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, msg);
}

static esp_err_t ota_update_post_handler(httpd_req_t *req)
{
    char buf[1024];
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = NULL;

    ESP_LOGI(TAG, "Starting HTTP POST OTA...");

    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "No passive partition for OTA found");
        send_ota_error(req, "No passive OTA partition found");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Writing OTA to partition %s at offset 0x%08x",
             update_partition->label, (unsigned int)update_partition->address);

    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        send_ota_error(req, "esp_ota_begin failed");
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    int received = 0;

    while (remaining > 0) {
        int read_len = sizeof(buf);
        if (remaining < read_len) {
            read_len = remaining;
        }
        int recv_len = httpd_req_recv(req, buf, read_len);
        if (recv_len <= 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "Socket receive error or timeout");
            esp_ota_abort(update_handle);
            send_ota_error(req, "Socket error");
            return ESP_FAIL;
        }

        err = esp_ota_write(update_handle, (const void *)buf, recv_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
            esp_ota_abort(update_handle);
            send_ota_error(req, "esp_ota_write failed");
            return ESP_FAIL;
        }

        remaining -= recv_len;
        received += recv_len;
    }

    ESP_LOGI(TAG, "Total bytes received: %d", received);

    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed (%s)", esp_err_to_name(err));
        send_ota_error(req, "esp_ota_end failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)", esp_err_to_name(err));
        send_ota_error(req, "esp_ota_set_boot_partition failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA upload complete. Rebooting...");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "success");

    // Defer reboot slightly to allow connection to close cleanly
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK;
}

static esp_err_t start_http_server(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12; // Increase limit to allow registering 9 handlers

    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        return err;
    }

    httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &root);

    httpd_uri_t cmd = {
        .uri = "/cmd",
        .method = HTTP_GET,
        .handler = cmd_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &cmd);

    httpd_uri_t status = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &status);

    httpd_uri_t ota_update = {
        .uri = "/ota/update",
        .method = HTTP_POST,
        .handler = ota_update_post_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &ota_update);

    httpd_uri_t ota_options = {
        .uri = "/ota/update",
        .method = HTTP_OPTIONS,
        .handler = options_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &ota_options);

    httpd_uri_t cam_ping = {
        .uri = "/cam_ping",
        .method = HTTP_GET,
        .handler = cam_ping_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &cam_ping);

    httpd_uri_t alert_uri = {
        .uri = "/alert",
        .method = HTTP_POST,
        .handler = alert_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &alert_uri);

    httpd_uri_t set_node = {
        .uri = "/set_node",
        .method = HTTP_GET,
        .handler = set_node_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &set_node);

    httpd_uri_t set_calibration = {
        .uri = "/set_calibration",
        .method = HTTP_GET,
        .handler = set_calibration_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &set_calibration);

    return ESP_OK;
}

esp_err_t wifi_server_start(wifi_server_command_cb_t command_cb)
{
    s_command_cb = command_cb;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASSWORD,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    if (strlen(WIFI_AP_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi AP started: SSID=%s password=%s", WIFI_AP_SSID, WIFI_AP_PASSWORD);
    return start_http_server();
}
