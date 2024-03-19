// https://nestedsoftware.com/2018/03/20/calculating-a-moving-average-on-streaming-data-5a7k.22879.html

pub struct MovingAverage
{
    count: u64,
    mean: f64,
}

pub struct SlidingAverage
{
    count: usize,
    batch: u64,
    buffer_vec: Vec<f64>,
    buffer_len: usize,
}

impl Default for MovingAverage
{
    fn default() -> Self
    {
        MovingAverage {
            count: 0,
            mean: 0.0,
        }
    }
}

impl MovingAverage
{
    pub fn update(&mut self, new_value: f64)
    {
        self.count += 1;

        let differential = (new_value - self.mean) / (self.count as f64);

        self.mean += differential;
    }
    pub fn reset(&mut self)
    {
        self.count = 0;
        self.mean = 0.0;
    }

    pub fn mean(&self) -> f64
    {
        self.mean
    }
    pub fn count(&self) -> u64
    {
        self.count
    }
}

impl Default for SlidingAverage
{
    fn default() -> Self
    {
        Self {
            count: 0,
            batch: 0,
            buffer_vec: vec![0.0; 20],
            buffer_len: 20,
        }
    }
}
impl SlidingAverage
{
    pub fn new(buffer_len: usize) -> Self
    {
        let mut this = Self {
            count: 0,
            batch: 0,
            buffer_vec: vec![],
            buffer_len: 0,
        };

        this.set(buffer_len);
        this
    }
    pub fn reset(&mut self)
    {
        self.count = 0;
        self.batch = 0;
    }

    pub fn update(&mut self, new_value: f64)
    {
        self.buffer_vec[self.count] = new_value;
        self.count = (self.count + 1) % self.buffer_len;
        if self.count == 0 {
            self.batch += 1;
        }
    }
    pub fn fill(&mut self, new_value: f64)
    {
        self.buffer_vec.fill(new_value);
        self.batch += 1;
        self.count = 0;
    }

    pub fn set(&mut self, buffer_len: usize)
    {
        self.buffer_len = buffer_len;
        self.buffer_vec.resize(buffer_len, 0.0);
    }

    pub fn mean(&self) -> (bool, f64)
    {
        let mean: f64 = self.buffer_vec.iter().sum::<f64>() / (self.buffer_len as f64);
        (self.batch > 0, mean)
    }
}

#[cfg(test)]
mod test
{
    use crate::common::statistic::{MovingAverage, SlidingAverage};
    use rand::Rng;
    #[test]
    fn test()
    {
        let mut moving_average = MovingAverage::default();

        for i in 0..10 {
            let i = i as f64 * 0.5;
            moving_average.update(i);
            println!("mean: {}", moving_average.mean());
        }

        let mut rng = rand::thread_rng();
        moving_average.reset();

        let mut sliding: SlidingAverage = SlidingAverage::default();
        sliding.set(30);

        for i in 0..100 {
            let v = rng.gen_range(-0.02..0.02);

            moving_average.update(v);
            sliding.update(v);
            println!("moving_average mean: {}", moving_average.mean());
            println!("sliding mean: {:?}", sliding.mean());
        }
    }
}
