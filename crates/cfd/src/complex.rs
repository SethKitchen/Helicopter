//! A minimal `std`-only complex number — just what the Joukowski conformal map and
//! its potential flow need (the workspace forbids external crates).

use std::ops::{Add, Div, Mul, Sub};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct C {
    pub re: f64,
    pub im: f64,
}

impl C {
    pub fn new(re: f64, im: f64) -> Self {
        C { re, im }
    }

    /// Scalar multiply `s·z`.
    pub fn scale(self, s: f64) -> C {
        C::new(self.re * s, self.im * s)
    }

    pub fn norm_sqr(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    pub fn abs(self) -> f64 {
        self.norm_sqr().sqrt()
    }

    /// Reciprocal `1/z`.
    pub fn recip(self) -> C {
        let d = self.norm_sqr();
        C::new(self.re / d, -self.im / d)
    }

    /// `exp(iθ)` (unit complex at angle θ).
    pub fn expi(theta: f64) -> C {
        C::new(theta.cos(), theta.sin())
    }
}

impl Add for C {
    type Output = C;
    fn add(self, o: C) -> C {
        C::new(self.re + o.re, self.im + o.im)
    }
}

impl Sub for C {
    type Output = C;
    fn sub(self, o: C) -> C {
        C::new(self.re - o.re, self.im - o.im)
    }
}

impl Mul for C {
    type Output = C;
    fn mul(self, o: C) -> C {
        C::new(self.re * o.re - self.im * o.im, self.re * o.im + self.im * o.re)
    }
}

impl Div for C {
    type Output = C;
    fn div(self, o: C) -> C {
        let d = o.norm_sqr();
        C::new((self.re * o.re + self.im * o.im) / d, (self.im * o.re - self.re * o.im) / d)
    }
}
