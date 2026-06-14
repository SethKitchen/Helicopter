//! A minimal complex number — just enough for the eigenvalue root finder.

use std::ops::{Add, Div, Mul, Sub};

/// Complex number.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Complex {
    /// Real part.
    pub re: f64,
    /// Imaginary part.
    pub im: f64,
}

impl Complex {
    /// New complex `re + im·i`.
    pub fn new(re: f64, im: f64) -> Self {
        Complex { re, im }
    }
    /// Zero.
    pub fn zero() -> Self {
        Complex { re: 0.0, im: 0.0 }
    }
    /// Real number as complex.
    pub fn real(re: f64) -> Self {
        Complex { re, im: 0.0 }
    }
    /// Magnitude.
    pub fn abs(&self) -> f64 {
        self.re.hypot(self.im)
    }
    /// Integer power by repeated multiplication.
    pub fn powi(self, n: u32) -> Self {
        let mut acc = Complex::real(1.0);
        for _ in 0..n {
            acc = acc * self;
        }
        acc
    }
}

impl Add for Complex {
    type Output = Complex;
    fn add(self, o: Complex) -> Complex {
        Complex::new(self.re + o.re, self.im + o.im)
    }
}
impl Sub for Complex {
    type Output = Complex;
    fn sub(self, o: Complex) -> Complex {
        Complex::new(self.re - o.re, self.im - o.im)
    }
}
impl Mul for Complex {
    type Output = Complex;
    fn mul(self, o: Complex) -> Complex {
        Complex::new(
            self.re * o.re - self.im * o.im,
            self.re * o.im + self.im * o.re,
        )
    }
}
impl Div for Complex {
    type Output = Complex;
    fn div(self, o: Complex) -> Complex {
        let d = o.re * o.re + o.im * o.im;
        Complex::new(
            (self.re * o.re + self.im * o.im) / d,
            (self.im * o.re - self.re * o.im) / d,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arithmetic() {
        let a = Complex::new(1.0, 2.0);
        let b = Complex::new(3.0, -1.0);
        assert_eq!(a + b, Complex::new(4.0, 1.0));
        assert_eq!(a * b, Complex::new(5.0, 5.0));
        let q = (a * b) / b;
        assert!((q.re - 1.0).abs() < 1e-12 && (q.im - 2.0).abs() < 1e-12);
    }
}
