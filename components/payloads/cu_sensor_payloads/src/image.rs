use bincode::{Decode, Encode};
use std::ops::{Deref, DerefMut};

#[repr(align(4096))]
#[derive(Clone, Debug, Encode, Decode)]
pub struct CuImageBuffer<const S: usize> {
    pixels: Box<[u8]>,
}

impl<const S: usize> Default for CuImageBuffer<S> {
    fn default() -> Self {
        let pixels = Box::new_uninit_slice(S);
        Self {
            pixels: unsafe { pixels.assume_init() },
        }
    }
}

impl<const S: usize> Deref for CuImageBuffer<S> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &*self.pixels
    }
}

impl<const S: usize> DerefMut for CuImageBuffer<S> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut *self.pixels
    }
}

// impl<const WIDTH: usize, const HEIGHT: usize, const STRIDE: usize, P>
//     CuImage<WIDTH, HEIGHT, STRIDE, P>
// where
//     P: Pixel + Default + 'static,
//     P::Subpixel: Copy,
// {
//     /// Builds an ImageBuffer from the image crate backed by the CuImage's pixel data.
//     pub fn as_image_buffer(&self) -> ImageBuffer<P, &[P::Subpixel]> {
//         assert_eq!(
//             STRIDE, WIDTH,
//             "STRIDE must equal WIDTH for ImageBuffer compatibility."
//         );
//
//         let raw_pixels: &[P::Subpixel] = unsafe {
//             core::slice::from_raw_parts(
//                 self.pixels.as_ptr() as *const P::Subpixel,
//                 WIDTH * HEIGHT * P::CHANNEL_COUNT as usize,
//             )
//         };
//
//         ImageBuffer::from_raw(WIDTH as u32, HEIGHT as u32, raw_pixels)
//             .expect("Failed to create ImageBuffer with CuImage's pixel data.")
//     }
// }
