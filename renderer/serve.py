"""렌더러용 정적 서버. Windows의 `python -m http.server`는 .mjs를 text/plain으로 내보내 ES 모듈 로드가 막히므로 MIME 타입을 바로잡는다.

저장소 루트에서:  python renderer/serve.py            → http://127.0.0.1:8724/renderer/
"""
import http.server
import os
import sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8724


class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {
        **http.server.SimpleHTTPRequestHandler.extensions_map,
        '.js': 'text/javascript', '.mjs': 'text/javascript',
        '.json': 'application/json', '.bin': 'application/octet-stream',
    }

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()


if __name__ == '__main__':
    os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    print(f'http://127.0.0.1:{PORT}/renderer/')
    http.server.ThreadingHTTPServer(('127.0.0.1', PORT), Handler).serve_forever()
