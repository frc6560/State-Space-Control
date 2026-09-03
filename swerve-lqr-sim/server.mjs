import { createReadStream, existsSync, statSync } from "node:fs";
import { createServer } from "node:http";
import { extname, join, normalize } from "node:path";

const root = new URL(".", import.meta.url).pathname.replace(/^\/(.:\/)/, "$1");
const mime = { ".html":"text/html; charset=utf-8", ".css":"text/css; charset=utf-8", ".mjs":"text/javascript; charset=utf-8", ".json":"application/json; charset=utf-8", ".md":"text/markdown; charset=utf-8" };

createServer((request, response) => {
  const relative = decodeURIComponent(new URL(request.url, "http://localhost").pathname).replace(/^\/+/, "") || "index.html";
  const file = normalize(join(root, relative));
  if (!file.startsWith(normalize(root)) || !existsSync(file) || !statSync(file).isFile()) {
    response.writeHead(404, { "Content-Type":"text/plain; charset=utf-8" }); response.end("Not found"); return;
  }
  response.writeHead(200, { "Content-Type":mime[extname(file)] || "application/octet-stream", "Cache-Control":"no-store" });
  createReadStream(file).pipe(response);
}).listen(8080, "127.0.0.1", () => console.log("Swerve LQR simulation: http://localhost:8080"));
