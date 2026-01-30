use heapless::Vec;

pub trait VecExt<T> {
    fn push_or_panic(&mut self, item: T);
}

impl<T, const N: usize> VecExt<T> for Vec<T, N> {
    fn push_or_panic(&mut self, item: T) {
        if self.push(item).is_err() {
            panic!("Vector is full! Increase the capacity N (current: {N}).");
        }
    }
}