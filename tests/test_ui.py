"""Exercise all three label displays in a real browser, without ROS or a camera."""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import unittest


PAGE = Path(__file__).resolve().parents[1] / 'app/ui/index.html'
BROWSER = shutil.which('chromium') or shutil.which('google-chrome')
LABEL = '<b>降落点</b> &quot; "'


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/ui/':
            mime, body = 'text/html', PAGE.read_bytes()
        elif self.path == '/api/status':
            tag = {'id': 77, 'size_m': 0.289, 'label': LABEL}
            detection = dict(tag, configured=True, distance_m=1, position_m=[0, 0, 1],
                             rpy_deg=[0, 0, 0], edge_px=10)
            mime, body = 'application/json', json.dumps({
                'detections': [detection], 'rules': {'tags': [tag]},
            }).encode()
        elif self.path == '/':
            mime = 'text/html'
            body = ('''<iframe src="/ui/"></iframe><output>WAIT</output><script>
const label = LABEL;
const timer = setInterval(() => {
    const page = document.querySelector('iframe').contentDocument;
    const cell = page.querySelector('#detBody td:nth-child(2)');
    const title = page.querySelector('#fovSvg text');
    const field = page.querySelector('input[data-field="label"]');
    if (!cell || !title || !field) return;
    clearInterval(timer);
    const literal = cell.childElementCount === 0 && cell.textContent === label
        && title.childElementCount === 0 && title.textContent.includes(label) && field.value === label;
    const row = () => page.querySelector('#tagRows tr');
    const id = () => row().querySelector('[data-field="id"]');
    id().value = '23';
    id().dispatchEvent(new Event('change', {bubbles: true}));
    field.value = 'edited';
    field.dispatchEvent(new Event('change', {bubbles: true}));
    page.querySelector('#addTagBtn').click();
    const edited = id().value === '23' && row().querySelector('[data-field="label"]').value === 'edited';
    row().querySelector('[data-del]').click();
    const deleted = page.querySelector('#tagRows').rows.length === 1 && id().value === '1';
    document.querySelector('output').textContent = literal && edited && deleted ? 'PASS' : 'FAIL';
}, 50);</script>'''.replace('LABEL', json.dumps(LABEL))).encode()
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', mime + '; charset=utf-8')
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass


class TagLabelTest(unittest.TestCase):
    @unittest.skipUnless(BROWSER, 'requires Chromium or Chrome')
    def test_label_is_literal_in_table_svg_and_editor(self):
        with ThreadingHTTPServer(('127.0.0.1', 0), Handler) as server, tempfile.TemporaryDirectory() as profile:
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                result = subprocess.run([
                    BROWSER, '--headless', '--disable-gpu', '--disable-background-networking',
                    f'--user-data-dir={profile}', '--virtual-time-budget=3000', '--dump-dom',
                    f'http://127.0.0.1:{server.server_port}/',
                ], capture_output=True, text=True, timeout=20, check=True)
                self.assertIn('<output>PASS</output>', result.stdout)
            finally:
                server.shutdown()
                thread.join()
