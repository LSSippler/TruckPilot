import http.server
import os

class HealthHandler(http.server.SimpleHTTPRequestHandler):
    def do_HEAD(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/plain")
        self.end_headers()

    def do_GET(self):
        if self.path == "/health":
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(b"OK\n")
        else:
            self.send_response(404)
            self.end_headers()

if __name__ == "__main__":
    addr = ("0.0.0.0", 8080)
    httpd = http.server.HTTPServer(addr, HealthHandler)
    print(f"Health endpoint lauscht auf http://0.0.0.0:8080")
    httpd.serve_forever()
