//! A tiny zero-dependency static HTTP server for the web design studio.
//!
//! `std::net` only — thread-per-connection, GET of files under a served root,
//! with a small extension→MIME map and path-traversal guard. Enough to serve the
//! studio's HTML/CSS/JS/JSON locally; not a production server (no keep-alive,
//! ranges, or compression — none of which the local UI needs).

use std::fs;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::path::{Component, Path, PathBuf};
use std::thread;

/// Serve `root` over HTTP on `127.0.0.1:port` until the process is killed.
/// Blocks. Prints the URL once bound.
pub fn serve(root: PathBuf, port: u16) -> std::io::Result<()> {
    let listener = TcpListener::bind(("127.0.0.1", port))?;
    println!("\n  ▶ design studio live at  http://127.0.0.1:{port}/");
    println!("    (serving {}; Ctrl-C to stop)\n", root.display());
    for stream in listener.incoming() {
        match stream {
            Ok(s) => {
                let root = root.clone();
                thread::spawn(move || {
                    let _ = handle(s, &root);
                });
            }
            Err(e) => eprintln!("  connection error: {e}"),
        }
    }
    Ok(())
}

fn handle(mut stream: TcpStream, root: &Path) -> std::io::Result<()> {
    // Read just the request line + headers (small; the UI issues simple GETs).
    let mut buf = [0u8; 8192];
    let n = stream.read(&mut buf)?;
    let req = String::from_utf8_lossy(&buf[..n]);
    let path = req
        .lines()
        .next()
        .and_then(|l| l.split_whitespace().nth(1))
        .unwrap_or("/");

    // Strip query string, default to index.html.
    let raw = path.split('?').next().unwrap_or("/");
    let rel = if raw == "/" { "/index.html" } else { raw };

    match resolve(root, rel) {
        Some(file) => match fs::read(&file) {
            Ok(body) => write_response(&mut stream, 200, "OK", mime(&file), &body),
            Err(_) => write_response(&mut stream, 404, "Not Found", "text/plain", b"404"),
        },
        None => write_response(&mut stream, 403, "Forbidden", "text/plain", b"403"),
    }
}

/// Resolve a URL path to a file under `root`, rejecting traversal (`..`, absolute
/// reroot). Returns `None` if the path escapes the served root.
fn resolve(root: &Path, rel: &str) -> Option<PathBuf> {
    let trimmed = rel.trim_start_matches('/');
    let candidate = Path::new(trimmed);
    // Reject any component that isn't a plain name.
    for c in candidate.components() {
        match c {
            Component::Normal(_) => {}
            _ => return None,
        }
    }
    Some(root.join(candidate))
}

fn mime(p: &Path) -> &'static str {
    match p.extension().and_then(|e| e.to_str()) {
        Some("html") => "text/html; charset=utf-8",
        Some("css") => "text/css; charset=utf-8",
        Some("js") | Some("mjs") => "text/javascript; charset=utf-8",
        Some("json") => "application/json; charset=utf-8",
        Some("svg") => "image/svg+xml",
        Some("stl") => "model/stl",
        Some("step") | Some("stp") => "application/step",
        Some("png") => "image/png",
        Some("woff2") => "font/woff2",
        _ => "application/octet-stream",
    }
}

fn write_response(
    stream: &mut TcpStream,
    code: u16,
    status: &str,
    content_type: &str,
    body: &[u8],
) -> std::io::Result<()> {
    let header = format!(
        "HTTP/1.1 {code} {status}\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n",
        body.len()
    );
    stream.write_all(header.as_bytes())?;
    stream.write_all(body)?;
    stream.flush()
}
