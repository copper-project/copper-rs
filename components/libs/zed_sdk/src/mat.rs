use std::ffi::c_void;
use std::marker::PhantomData;
use std::mem::size_of;
use std::slice;

use crate::error::{Error, ErrorCode, Result};
use crate::sys;
use crate::types::{Bgr8, Bgra8, MemoryType, Point3Color, Resolution, Rgba8, Vec2f, Vec3f, Vec4f};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MatType {
    F32C1,
    F32C2,
    F32C3,
    F32C4,
    U8C1,
    U8C2,
    U8C3,
    U8C4,
    U16C1,
    S8C4,
    Unknown(i32),
}

impl MatType {
    pub(crate) fn as_raw(self) -> sys::SL_MAT_TYPE {
        match self {
            Self::F32C1 => sys::SL_MAT_TYPE::SL_MAT_TYPE_F32_C1,
            Self::F32C2 => sys::SL_MAT_TYPE::SL_MAT_TYPE_F32_C2,
            Self::F32C3 => sys::SL_MAT_TYPE::SL_MAT_TYPE_F32_C3,
            Self::F32C4 => sys::SL_MAT_TYPE::SL_MAT_TYPE_F32_C4,
            Self::U8C1 => sys::SL_MAT_TYPE::SL_MAT_TYPE_U8_C1,
            Self::U8C2 => sys::SL_MAT_TYPE::SL_MAT_TYPE_U8_C2,
            Self::U8C3 => sys::SL_MAT_TYPE::SL_MAT_TYPE_U8_C3,
            Self::U8C4 => sys::SL_MAT_TYPE::SL_MAT_TYPE_U8_C4,
            Self::U16C1 => sys::SL_MAT_TYPE::SL_MAT_TYPE_U16_C1,
            Self::S8C4 => sys::SL_MAT_TYPE::SL_MAT_TYPE_S8_C4,
            Self::Unknown(raw) => panic!("unknown SL_MAT_TYPE value {raw}"),
        }
    }

    pub(crate) fn from_raw(raw: i32) -> Self {
        match raw {
            0 => Self::F32C1,
            1 => Self::F32C2,
            2 => Self::F32C3,
            3 => Self::F32C4,
            4 => Self::U8C1,
            5 => Self::U8C2,
            6 => Self::U8C3,
            7 => Self::U8C4,
            8 => Self::U16C1,
            9 => Self::S8C4,
            _ => Self::Unknown(raw),
        }
    }
}

pub unsafe trait MatElement: Copy + 'static {
    const MAT_TYPE: MatType;
}

// SAFETY: These Rust types are repr(C) or primitive layouts matching the ZED SDK mat element type.
unsafe impl MatElement for u8 {
    const MAT_TYPE: MatType = MatType::U8C1;
}

// SAFETY: `u16` matches ZED's 16-bit scalar pixel layout.
unsafe impl MatElement for u16 {
    const MAT_TYPE: MatType = MatType::U16C1;
}

// SAFETY: `f32` matches ZED's 32-bit scalar pixel layout.
unsafe impl MatElement for f32 {
    const MAT_TYPE: MatType = MatType::F32C1;
}

// SAFETY: `Vec2f` is repr(C) with two `f32` lanes.
unsafe impl MatElement for Vec2f {
    const MAT_TYPE: MatType = MatType::F32C2;
}

// SAFETY: `Vec3f` is repr(C) with three `f32` lanes.
unsafe impl MatElement for Vec3f {
    const MAT_TYPE: MatType = MatType::F32C3;
}

// SAFETY: `Vec4f` is repr(C) with four `f32` lanes.
unsafe impl MatElement for Vec4f {
    const MAT_TYPE: MatType = MatType::F32C4;
}

// SAFETY: `Point3Color` is repr(C) and matches the SDK's float4 XYZRGBA layout.
unsafe impl MatElement for Point3Color {
    const MAT_TYPE: MatType = MatType::F32C4;
}

// SAFETY: `Bgr8` is repr(C) with three `u8` lanes.
unsafe impl MatElement for Bgr8 {
    const MAT_TYPE: MatType = MatType::U8C3;
}

// SAFETY: `Bgra8` is repr(C) with four `u8` lanes.
unsafe impl MatElement for Bgra8 {
    const MAT_TYPE: MatType = MatType::U8C4;
}

// SAFETY: `Rgba8` is repr(C) with four `u8` lanes.
unsafe impl MatElement for Rgba8 {
    const MAT_TYPE: MatType = MatType::U8C4;
}

pub struct Mat<T: MatElement> {
    ptr: *mut c_void,
    memory: MemoryType,
    marker: PhantomData<T>,
}

impl<T: MatElement> Mat<T> {
    pub fn new(resolution: Resolution, memory: MemoryType) -> Result<Self> {
        let width = resolution.width_i32()?;
        let height = resolution.height_i32()?;
        let ptr =
            unsafe { sys::sl_mat_create_new(width, height, T::MAT_TYPE.as_raw(), memory.as_raw()) };
        if ptr.is_null() {
            return Err(Error::NullMatAllocation {
                width: resolution.width(),
                height: resolution.height(),
                mat_type: T::MAT_TYPE,
                memory,
            });
        }

        Ok(Self {
            ptr,
            memory,
            marker: PhantomData,
        })
    }

    pub fn new_cpu(resolution: Resolution) -> Result<Self> {
        Self::new(resolution, MemoryType::Cpu)
    }

    pub fn resolution(&self) -> Result<Resolution> {
        let width = unsafe { sys::sl_mat_get_width(self.ptr) };
        let height = unsafe { sys::sl_mat_get_height(self.ptr) };
        Resolution::try_from_raw(width, height, "mat")
    }

    pub fn memory_type(&self) -> MemoryType {
        self.memory
    }

    pub fn mat_type(&self) -> MatType {
        let raw = unsafe { sys::sl_mat_get_data_type(self.ptr) };
        MatType::from_raw(raw)
    }

    pub fn pixel_size(&self) -> usize {
        unsafe { sys::sl_mat_get_pixel_bytes(self.ptr) as usize }
    }

    pub fn view(&self) -> Result<MatView<'_, T>> {
        let metadata = self.view_metadata()?;
        let ptr = unsafe { sys::sl_mat_get_ptr(self.ptr, sys::SL_MEM::SL_MEM_CPU) }.cast::<T>();
        if ptr.is_null() {
            return Err(Error::MatPointerUnavailable);
        }

        let data = unsafe { slice::from_raw_parts(ptr.cast_const(), metadata.buffer_len_elems) };
        Ok(MatView {
            width: metadata.width,
            height: metadata.height,
            stride_elems: metadata.stride_elems,
            data,
        })
    }

    pub fn view_mut(&mut self) -> Result<MatViewMut<'_, T>> {
        let metadata = self.view_metadata()?;
        let ptr = unsafe { sys::sl_mat_get_ptr(self.ptr, sys::SL_MEM::SL_MEM_CPU) }.cast::<T>();
        if ptr.is_null() {
            return Err(Error::MatPointerUnavailable);
        }

        let data = unsafe { slice::from_raw_parts_mut(ptr, metadata.buffer_len_elems) };
        Ok(MatViewMut {
            width: metadata.width,
            height: metadata.height,
            stride_elems: metadata.stride_elems,
            data,
        })
    }

    pub fn sync_cpu_from_gpu(&mut self) -> Result<()> {
        let code = unsafe { sys::sl_mat_update_cpu_from_gpu(self.ptr) };
        let code = ErrorCode::from_raw(code);
        if code.is_success() {
            Ok(())
        } else {
            Err(Error::Sdk {
                operation: "sl_mat_update_cpu_from_gpu",
                code,
            })
        }
    }

    fn view_metadata(&self) -> Result<ViewMetadata> {
        let actual_memory = unsafe { sys::sl_mat_get_memory_type(self.ptr) };
        let actual_memory =
            MemoryType::from_raw(actual_memory).ok_or(Error::MatCpuAccessUnavailable {
                memory: self.memory,
            })?;
        if !actual_memory.exposes_cpu() {
            return Err(Error::MatCpuAccessUnavailable {
                memory: actual_memory,
            });
        }

        let actual_type = self.mat_type();
        if actual_type != T::MAT_TYPE {
            return Err(Error::MatTypeMismatch {
                expected: T::MAT_TYPE,
                actual: actual_type,
            });
        }

        let resolution = self.resolution()?;
        let width = resolution.width() as usize;
        let height = resolution.height() as usize;
        let step_bytes = unsafe { sys::sl_mat_get_step_bytes(self.ptr, sys::SL_MEM::SL_MEM_CPU) };
        let step_bytes = usize::try_from(step_bytes).map_err(|_| Error::MatStrideMisaligned {
            step_bytes: 0,
            element_size: size_of::<T>(),
        })?;
        let element_size = size_of::<T>();
        if step_bytes % element_size != 0 {
            return Err(Error::MatStrideMisaligned {
                step_bytes,
                element_size,
            });
        }

        let stride_elems = step_bytes / element_size;
        let buffer_len_elems =
            stride_elems
                .checked_mul(height)
                .ok_or(Error::MatStrideMisaligned {
                    step_bytes,
                    element_size,
                })?;

        Ok(ViewMetadata {
            width,
            height,
            stride_elems,
            buffer_len_elems,
        })
    }

    pub(crate) fn as_raw_mut_ptr(&mut self) -> *mut c_void {
        self.ptr
    }
}

impl<T: MatElement> Drop for Mat<T> {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            unsafe { sys::sl_mat_free(self.ptr, self.memory.as_raw()) };
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct ViewMetadata {
    width: usize,
    height: usize,
    stride_elems: usize,
    buffer_len_elems: usize,
}

pub struct MatView<'a, T> {
    width: usize,
    height: usize,
    stride_elems: usize,
    data: &'a [T],
}

impl<'a, T> MatView<'a, T> {
    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn stride_elems(&self) -> usize {
        self.stride_elems
    }

    pub fn is_tightly_packed(&self) -> bool {
        self.stride_elems == self.width
    }

    pub fn row(&self, y: usize) -> Option<&'a [T]> {
        if y >= self.height {
            return None;
        }

        let start = y * self.stride_elems;
        let end = start + self.width;
        Some(&self.data[start..end])
    }

    pub fn rows(&self) -> impl ExactSizeIterator<Item = &[T]> + Clone + '_ {
        (0..self.height).map(move |row| {
            let start = row * self.stride_elems;
            let end = start + self.width;
            &self.data[start..end]
        })
    }

    pub fn get(&self, x: usize, y: usize) -> Option<&'a T> {
        self.row(y).and_then(|row| row.get(x))
    }

    pub fn as_slice(&self) -> Option<&'a [T]> {
        self.is_tightly_packed()
            .then_some(&self.data[..self.width * self.height])
    }

    pub fn as_padded_slice(&self) -> &'a [T] {
        self.data
    }

    pub fn as_bytes(&self) -> Option<&'a [u8]> {
        self.as_slice().map(|data| unsafe {
            slice::from_raw_parts(data.as_ptr().cast::<u8>(), std::mem::size_of_val(data))
        })
    }

    pub fn as_padded_bytes(&self) -> &'a [u8] {
        unsafe {
            slice::from_raw_parts(
                self.data.as_ptr().cast::<u8>(),
                std::mem::size_of_val(self.data),
            )
        }
    }
}

pub struct MatViewMut<'a, T> {
    width: usize,
    height: usize,
    stride_elems: usize,
    data: &'a mut [T],
}

impl<'a, T> MatViewMut<'a, T> {
    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn stride_elems(&self) -> usize {
        self.stride_elems
    }

    pub fn is_tightly_packed(&self) -> bool {
        self.stride_elems == self.width
    }

    pub fn row_mut(&mut self, y: usize) -> Option<&mut [T]> {
        if y >= self.height {
            return None;
        }

        let start = y * self.stride_elems;
        let end = start + self.width;
        Some(&mut self.data[start..end])
    }

    pub fn get_mut(&mut self, x: usize, y: usize) -> Option<&mut T> {
        self.row_mut(y).and_then(|row| row.get_mut(x))
    }

    pub fn as_mut_slice(&mut self) -> Option<&mut [T]> {
        let width = self.width;
        let height = self.height;
        self.is_tightly_packed()
            .then_some(&mut self.data[..width * height])
    }
}
