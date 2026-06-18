//! Minimal hand-written **SVG** builder (zero dependencies) — vector graphics for
//! the build instructions, so the swashplate linkage, the rotor-head load path and
//! the blade section are SHOWN, not just described in text.
//!
//! Same discipline as the STL/DXF writers: assemble the markup by hand and let the
//! tests check it is well-formed (header/footer, viewBox, expected elements). SVG
//! renders in any browser or imports into Inkscape/Illustrator — no toolchain needed.
//!
//! Convention: SVG y grows DOWNWARD; diagram code flips where an upright drawing is
//! wanted. Units are user pixels (≈ mm in the diagrams, scaled to fit the canvas).

use std::fmt::Write as _;

/// An SVG canvas being assembled.
pub struct Svg {
    body: String,
    width: f64,
    height: f64,
}

impl Svg {
    /// A new canvas `width × height` pixels.
    pub fn new(width: f64, height: f64) -> Self {
        Svg {
            body: String::new(),
            width,
            height,
        }
    }

    /// A straight line.
    pub fn line(&mut self, x1: f64, y1: f64, x2: f64, y2: f64, stroke: &str, w: f64) {
        let _ = writeln!(
            self.body,
            "  <line x1=\"{x1:.2}\" y1=\"{y1:.2}\" x2=\"{x2:.2}\" y2=\"{y2:.2}\" \
             stroke=\"{stroke}\" stroke-width=\"{w}\"/>"
        );
    }

    /// A rectangle (lightly rounded corners) with fill + stroke.
    pub fn rect(&mut self, x: f64, y: f64, w: f64, h: f64, fill: &str, stroke: &str) {
        let _ = writeln!(
            self.body,
            "  <rect x=\"{x:.2}\" y=\"{y:.2}\" width=\"{w:.2}\" height=\"{h:.2}\" rx=\"2\" \
             fill=\"{fill}\" stroke=\"{stroke}\" stroke-width=\"1\"/>"
        );
    }

    /// A circle.
    pub fn circle(&mut self, cx: f64, cy: f64, r: f64, fill: &str, stroke: &str) {
        let _ = writeln!(
            self.body,
            "  <circle cx=\"{cx:.2}\" cy=\"{cy:.2}\" r=\"{r:.2}\" fill=\"{fill}\" \
             stroke=\"{stroke}\" stroke-width=\"1\"/>"
        );
    }

    /// A polyline / polygon (set `closed` for a filled polygon).
    pub fn poly(&mut self, pts: &[(f64, f64)], fill: &str, stroke: &str, closed: bool) {
        let tag = if closed { "polygon" } else { "polyline" };
        let mut d = String::new();
        for (x, y) in pts {
            let _ = write!(d, "{x:.2},{y:.2} ");
        }
        let _ = writeln!(
            self.body,
            "  <{tag} points=\"{}\" fill=\"{fill}\" stroke=\"{stroke}\" stroke-width=\"1.2\"/>",
            d.trim_end()
        );
    }

    /// Text. `anchor` ∈ {start,middle,end}.
    pub fn text(&mut self, x: f64, y: f64, s: &str, size: f64, anchor: &str, fill: &str) {
        let _ = writeln!(
            self.body,
            "  <text x=\"{x:.2}\" y=\"{y:.2}\" font-family=\"sans-serif\" font-size=\"{size}\" \
             text-anchor=\"{anchor}\" fill=\"{fill}\">{}</text>",
            escape(s)
        );
    }

    /// An arrow from (x1,y1) to (x2,y2) with a small arrowhead at the end.
    pub fn arrow(&mut self, x1: f64, y1: f64, x2: f64, y2: f64, color: &str) {
        self.line(x1, y1, x2, y2, color, 1.5);
        let (dx, dy) = (x2 - x1, y2 - y1);
        let len = (dx * dx + dy * dy).sqrt().max(1e-6);
        let (ux, uy) = (dx / len, dy / len);
        let head = 7.0;
        // Two barbs at ±25° from the reversed direction.
        for sgn in [-1.0_f64, 1.0] {
            let ang = 0.43 * sgn; // ~25°
            let (cx, sx) = (ang.cos(), ang.sin());
            let bx = x2 - head * (ux * cx - uy * sx);
            let by = y2 - head * (uy * cx + ux * sx);
            self.line(x2, y2, bx, by, color, 1.5);
        }
    }

    /// A dimension line with end ticks and a centered label (horizontal or vertical).
    pub fn dim(&mut self, x1: f64, y1: f64, x2: f64, y2: f64, label: &str) {
        let gray = "#666";
        self.line(x1, y1, x2, y2, gray, 0.8);
        // End ticks perpendicular-ish (short).
        for (x, y) in [(x1, y1), (x2, y2)] {
            self.line(x - 3.0, y - 3.0, x + 3.0, y + 3.0, gray, 0.8);
        }
        let (mx, my) = (0.5 * (x1 + x2), 0.5 * (y1 + y2));
        self.text(mx, my - 4.0, label, 11.0, "middle", gray);
    }

    /// Render the complete SVG document with a title bar.
    pub fn render(&self, title: &str) -> String {
        let mut s = String::new();
        let _ = writeln!(
            s,
            "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 {:.0} {:.0}\" \
             width=\"{:.0}\" height=\"{:.0}\">",
            self.width, self.height, self.width, self.height
        );
        let _ = writeln!(
            s,
            "  <rect x=\"0\" y=\"0\" width=\"{:.0}\" height=\"{:.0}\" fill=\"white\"/>",
            self.width, self.height
        );
        let _ = writeln!(
            s,
            "  <text x=\"10\" y=\"22\" font-family=\"sans-serif\" font-size=\"16\" \
             font-weight=\"bold\" fill=\"#111\">{}</text>",
            escape(title)
        );
        s.push_str(&self.body);
        s.push_str("</svg>\n");
        s
    }
}

/// Escape the five XML metacharacters so labels can't break the markup.
fn escape(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn renders_well_formed_svg() {
        let mut g = Svg::new(200.0, 100.0);
        g.rect(10.0, 10.0, 50.0, 30.0, "none", "black");
        g.circle(100.0, 50.0, 8.0, "red", "black");
        g.line(0.0, 0.0, 200.0, 100.0, "blue", 1.0);
        g.arrow(20.0, 80.0, 80.0, 80.0, "green");
        g.dim(10.0, 90.0, 60.0, 90.0, "50 mm");
        g.text(100.0, 90.0, "label", 12.0, "middle", "black");
        let svg = g.render("Test");
        assert!(svg.starts_with("<svg"));
        assert!(svg.contains("viewBox=\"0 0 200 100\""));
        assert!(svg.trim_end().ends_with("</svg>"));
        assert!(svg.contains("<circle") && svg.contains("<line") && svg.contains("<text"));
        assert!(svg.contains("Test"));
    }

    #[test]
    fn escapes_xml_metacharacters() {
        let mut g = Svg::new(50.0, 50.0);
        g.text(0.0, 0.0, "a<b & c>\"d\"", 10.0, "start", "black");
        let svg = g.render("t");
        assert!(svg.contains("a&lt;b &amp; c&gt;"));
        assert!(!svg.contains("a<b"));
    }
}
