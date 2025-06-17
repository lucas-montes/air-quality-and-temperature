pub struct StrWriter<'a> {
    pub buffer: &'a mut [u8],
    pub pos: usize,
}

impl<'a> core::fmt::Write for StrWriter<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let len = bytes.len();

        if self.pos + len > self.buffer.len() {
            return Err(core::fmt::Error);
        }

        self.buffer[self.pos..self.pos + len].copy_from_slice(bytes);
        self.pos += len;
        Ok(())
    }
}

impl<'a> StrWriter<'a> {
    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buffer[..self.pos]).unwrap_or("")
    }
}

pub struct RollingMedian<'a, const N: usize, T: Copy + PartialOrd + Default> {
    buffer: [T; N],
    index: usize,
    filled: bool,
    title: &'a str,
}

impl<'a,const N: usize, T: Copy + PartialOrd + Default> RollingMedian<'a, N, T> {
    pub fn new(title: &'a str,) -> Self {
        Self {
            buffer: [T::default(); N],
            index: 0,
            filled: false,
            title
        }
    }

    pub fn update(&mut self, value: T) {
        self.buffer[self.index] = value;
        self.index = (self.index + 1) % N;
        if self.index == 0 {
            self.filled = true;
        }
    }

    pub fn median(&self) -> T {
        let mut sorted = self.buffer;
        let count = if self.filled { N } else { self.index };

        for i in 1..count {
            let mut j = i;
            while j > 0 && sorted[j] < sorted[j - 1] {
                sorted.swap(j, j - 1);
                j -= 1;
            }
        }

        sorted[count / 2]
    }
}

impl<'a,const N: usize, T: core::fmt::Display + Copy + PartialOrd + Default> core::fmt::Display for RollingMedian<'a, N, T> {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "{}: {}", self.title, self.median())
    }
}
