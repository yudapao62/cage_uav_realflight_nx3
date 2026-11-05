#!/usr/bin/env python3
"""Always-on ground supervisor. It never arms or commands the aircraft."""
import json
import os
import subprocess
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HOST = '0.0.0.0'
PORT = 8080
CORE = 'http://127.0.0.1:8081'
DATA_CORE = 'http://127.0.0.1:8090'
STATE_DIR = '/home/nx3/.cage_realflight_stack'
PID_FILE = os.path.join(STATE_DIR, 'pids')
START_LOG = os.path.join(STATE_DIR, 'logs', 'supervisor-start.log')
PROFILE_FILE = os.path.join(STATE_DIR, 'input_profile')
START_SCRIPT = '/home/nx3/cage_uav_ws/src/cage_realflight/scripts/start_realflight_stack.sh'
STOP_SCRIPT = '/home/nx3/cage_uav_ws/src/cage_realflight/scripts/stop_realflight_stack.sh'
DATA_START_SCRIPT = '/home/nx3/cage_uav_ws/src/cage_site_db/scripts/start_site_db.sh'

lock = threading.Lock()
data_lock = threading.Lock()
launcher = None


def core_request(path='/api/status', method='GET', body=None, timeout=2):
    data = None if body is None else json.dumps(body).encode('utf-8')
    req = urllib.request.Request(CORE+path, data=data, method=method,
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=timeout) as response:
        return response.status, response.headers.get('Content-Type', 'application/octet-stream'), response.read()


def data_request(path='/api/site-db/health', method='GET', body=None, timeout=2):
    data = None if body is None else json.dumps(body).encode('utf-8')
    req = urllib.request.Request(DATA_CORE+path, data=data, method=method,
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=timeout) as response:
        return response.status, response.headers.get('Content-Type', 'application/octet-stream'), response.read()


def ensure_data_service():
    try:
        data_request(timeout=0.5)
        return True
    except Exception:
        pass
    with data_lock:
        try:
            data_request(timeout=0.5)
            return True
        except Exception:
            pass
        try:
            subprocess.run([DATA_START_SCRIPT], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           timeout=10, check=True)
            data_request(timeout=2)
            return True
        except Exception:
            return False


def recorded_processes():
    result = []
    try:
        with open(PID_FILE) as handle:
            for line in handle:
                parts = line.split(maxsplit=1)
                if not parts or not parts[0].isdigit(): continue
                pid = int(parts[0]); label = parts[1].strip() if len(parts) > 1 else ''
                try: os.kill(pid, 0); alive = True
                except OSError: alive = False
                result.append({'pid': pid, 'label': label, 'alive': alive})
    except OSError:
        pass
    return result


def tail(path, lines=80):
    try:
        with open(path, errors='replace') as handle:
            data = ''.join(handle.readlines()[-lines:]).replace('\x00', '')
            return data[-12000:]
    except OSError: return ''


def status():
    global launcher
    try:
        _code, _ctype, body = core_request()
        core = json.loads(body.decode('utf-8')); available = True
    except Exception:
        core = {}; available = False
    processes = recorded_processes()
    try: data_request(timeout=0.5); data_available = True
    except Exception: data_available = False
    input_profile = core.get('input_profile') if available else None
    if not input_profile and any(item['alive'] for item in processes):
        try:
            with open(PROFILE_FILE) as handle:
                input_profile = handle.read().strip() or None
        except OSError:
            pass
    return {'supervisor': 'RUNNING', 'core_available': available, 'data_service_available': data_available,
            'input_profile': input_profile,
            'stack_running': any(item['alive'] for item in processes),
            'launcher_running': launcher is not None and launcher.poll() is None,
            'processes': processes, 'core': core,
            'start_log': tail(START_LOG), 'cage_log': tail(os.path.join(STATE_DIR, 'logs', 'cage.log'), 40),
            'gps_log': tail(os.path.join(STATE_DIR, 'logs', 'gps.log'), 30),
            'mavros_log': tail(os.path.join(STATE_DIR, 'logs', 'mavros.log'), 30)}


def start_stack(input_profile, feeding=True, rviz=False):
    global launcher
    input_profile = str(input_profile or '').strip().upper()
    if input_profile not in ('GROUND_GPS', 'PHOTO_COORDS'):
        return False, 'INPUT_PROFILE_REQUIRED'
    if not isinstance(feeding, bool) or not isinstance(rviz, bool):
        return False, 'FEEDING_AND_RVIZ_MUST_BE_BOOLEAN'
    with lock:
        state = status()
        if state['core_available'] or state['stack_running'] or state['launcher_running']:
            return False, 'STACK_ALREADY_RUNNING_OR_STARTING'
        os.makedirs(os.path.dirname(START_LOG), exist_ok=True)
        log = open(START_LOG, 'wb', buffering=0)
        command = [START_SCRIPT, '--input-profile', input_profile.lower(),
                   '--geometry', 'circular', '--feeding', str(feeding).lower(),
                   '--rviz', str(rviz).lower(), '--web', 'true',
                   '--web-port', '8081']
        launcher = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
        return True, ''


SUPERVISOR_PAGE = '''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>CAGE UAV 系统启动</title>
<style>body{margin:0;background:#071019;color:#eaf2f8;font:15px system-ui}main{max-width:920px;margin:5vh auto;padding:24px}.box{background:#101d29;border:1px solid #26394a;border-radius:14px;padding:22px}.profiles{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin:18px 0}.profile{border:2px solid #26394a;border-radius:10px;padding:14px;background:#09131c;cursor:pointer}.profile.active{border-color:#4eb5ff;background:#14334a}.profile input{margin-right:8px}button{background:#4eb5ff;border:0;border-radius:9px;padding:12px 18px;font-weight:800;cursor:pointer}button:disabled{opacity:.4}pre{background:#050b10;padding:14px;white-space:pre-wrap;max-height:42vh;overflow:auto;color:#b9cbd8}.bad{color:#ff6972}.ok{color:#2bd98f}.muted{color:#8fa5b5}a{color:#4eb5ff;font-weight:800}@media(max-width:700px){.profiles{grid-template-columns:1fr}}</style></head>
<body><main><div class="box"><h1>CAGE UAV 实飞系统</h1><p id="state">读取状态…</p>
<h2>初始化前选择坐标来源</h2><div class="profiles">
<label class="profile"><input type="radio" name="profile" value="GROUND_GPS"><b>双地面 GPS A/B</b><p class="muted">启动 /dev/ttyUSB1、GPS接收机、A/B几何和中点/对角网格任务。</p></label>
<label class="profile"><input type="radio" name="profile" value="PHOTO_COORDS"><b>照片/人工坐标（无地面GPS）</b><p class="muted">不检查 /dev/ttyUSB1，不启动A/B；使用照片EXIF或已审核人工经纬度。</p></label>
</div><label><input id="feeding" type="checkbox" checked> 启用真实投喂器</label>　<label><input id="rviz" type="checkbox"> 启动RViz</label>
<p><button id="start" onclick="go()" disabled>启动所选实飞系统</button>　<a href="/site-db/">网箱坐标数据库</a></p>
<p class="muted">每次停止系统后必须重新选择。飞控自身GPS在两种模式下都必须有效。</p><pre id="log"></pre></div></main>
<script>const radios=[...document.querySelectorAll('[name=profile]')],button=document.querySelector('#start');radios.forEach(r=>r.onchange=()=>{document.querySelectorAll('.profile').forEach(x=>x.classList.toggle('active',x.contains(r)&&r.checked));button.disabled=false});
async function poll(){let s=await(await fetch('/api/supervisor/status',{cache:'no-store'})).json(),running=s.launcher_running||s.stack_running;document.querySelector('#state').textContent=s.core_available?`系统可用 · ${s.input_profile||'UNKNOWN'}，正在进入控制台…`:s.launcher_running?'正在启动…':s.stack_running?`系统运行中 · ${s.input_profile||'UNKNOWN'}`:'系统未启动，请选择输入模式';document.querySelector('#state').className=s.core_available?'ok':'';button.disabled=running||!radios.some(r=>r.checked);document.querySelector('#log').textContent=s.start_log||s.gps_log||'暂无日志';if(s.core_available)location.reload()}
async function go(){let p=radios.find(r=>r.checked);if(!p)return;button.disabled=true;let r=await fetch('/api/supervisor/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({input_profile:p.value,feeding:document.querySelector('#feeding').checked,rviz:document.querySelector('#rviz').checked})}),j=await r.json();if(!r.ok){alert(j.error||'启动失败');button.disabled=false}poll()}poll();setInterval(poll,1500)</script></body></html>'''


class Handler(BaseHTTPRequestHandler):
    def reply(self, code, ctype, body):
        if isinstance(body, str): body = body.encode('utf-8')
        self.send_response(code); self.send_header('Content-Type', ctype)
        self.send_header('Cache-Control', 'no-store'); self.send_header('Content-Length', str(len(body)))
        self.end_headers(); self.wfile.write(body)

    def json_reply(self, value, code=200):
        self.reply(code, 'application/json', json.dumps(value).encode('utf-8'))

    def proxy(self, method, base=CORE):
        length = int(self.headers.get('Content-Length', '0'))
        raw = self.rfile.read(length) if length else None
        try:
            req = urllib.request.Request(base+self.path, data=raw, method=method,
                                         headers={'Content-Type': self.headers.get('Content-Type', 'application/json')})
            with urllib.request.urlopen(req, timeout=30) as response:
                self.reply(response.status, response.headers.get('Content-Type', 'application/octet-stream'), response.read())
        except urllib.error.HTTPError as exc:
            self.reply(exc.code, exc.headers.get('Content-Type', 'application/json'), exc.read())
        except Exception as exc:
            self.json_reply({'error': 'CORE_UNAVAILABLE', 'detail': str(exc)}, 503)

    def do_GET(self):
        if self.path == '/api/supervisor/status': return self.json_reply(status())
        if self.path == '/api/supervisor/profiles':
            return self.json_reply({'profiles': [
                {'id': 'GROUND_GPS', 'requires_ground_gps': True},
                {'id': 'PHOTO_COORDS', 'requires_ground_gps': False},
            ], 'selection_required': True})
        if self.path.startswith('/site-db') or self.path.startswith('/api/site-db'):
            if not ensure_data_service(): return self.json_reply({'error': 'DATA_SERVICE_UNAVAILABLE'}, 503)
            return self.proxy('GET', DATA_CORE)
        try: core_request(timeout=0.5); available = True
        except Exception: available = False
        if not available and self.path == '/': return self.reply(200, 'text/html; charset=utf-8', SUPERVISOR_PAGE)
        return self.proxy('GET')

    def do_POST(self):
        if self.path.startswith('/api/site-db'):
            if not ensure_data_service(): return self.json_reply({'error': 'DATA_SERVICE_UNAVAILABLE'}, 503)
            return self.proxy('POST', DATA_CORE)
        if self.path == '/api/supervisor/start':
            try:
                length = int(self.headers.get('Content-Length', '0'))
                payload = json.loads(self.rfile.read(length).decode('utf-8')) if length else {}
            except (TypeError, ValueError):
                return self.json_reply({'ok': False, 'error': 'INVALID_JSON'}, 400)
            ok, error = start_stack(payload.get('input_profile'),
                                    payload.get('feeding', True),
                                    payload.get('rviz', False))
            return self.json_reply({'ok': ok, 'error': error}, 200 if ok else 409)
        if self.path == '/api/supervisor/stop':
            try:
                _code, _ctype, body = core_request(); core = json.loads(body.decode('utf-8'))
            except Exception:
                return self.json_reply({'error': 'CANNOT_VERIFY_DISARMED'}, 409)
            if core.get('armed') is not False:
                return self.json_reply({'error': 'AIRCRAFT_ARMED_OR_UNKNOWN'}, 409)
            subprocess.Popen([STOP_SCRIPT], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             start_new_session=True)
            return self.json_reply({'ok': True})
        return self.proxy('POST')

    def log_message(self, fmt, *args): pass


if __name__ == '__main__':
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()
