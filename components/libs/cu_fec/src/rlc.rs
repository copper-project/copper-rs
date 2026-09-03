//! Bounded sliding-window RLC encoder and decoder.

use core::fmt;

use crate::{Field, RepairPayloadId, tinymt::TinyMt32};

const RFC_MAX_WINDOW_SYMBOLS: usize = 0x0fff;

/// A codec configuration error or an error encountered while processing a symbol.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    /// The symbol size is zero.
    EmptySymbol,
    /// The configured active symbol size exceeds the type's hard capacity.
    SymbolCapacityExceeded,
    /// A source or repair symbol does not have the configured size.
    SymbolSizeMismatch,
    /// The coding window size is zero.
    EmptyWindow,
    /// The coding window exceeds either the RFC limit or the type's hard capacity.
    WindowCapacityExceeded,
    /// The active decoder equation capacity is zero or exceeds its hard capacity.
    EquationCapacityExceeded,
    /// The supplied density threshold is greater than 15.
    InvalidDensityThreshold,
    /// GF(2) at full density requires a zero repair key on the sending side.
    RepairKeyMustBeZero,
    /// No source symbol is currently available for repair generation.
    EmptyEncodingWindow,
    /// A received repair window is larger than the configured decoding window.
    RepairWindowTooLarge,
    /// A symbol or repair window is older than the retained decoding window.
    WindowExpired,
    /// Sequence ordering is ambiguous at exactly half the `u32` sequence space.
    AmbiguousSequence,
    /// The bounded equation store has no room for another innovative equation.
    EquationStoreFull,
    /// Received data contradicts data already known to the decoder.
    ConflictingSource,
    /// A reduced equation has no variables but has a nonzero right-hand side.
    InconsistentEquation,
}

impl fmt::Display for Error {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(match self {
            Self::EmptySymbol => "symbol size must be nonzero",
            Self::SymbolCapacityExceeded => "symbol size exceeds the hard capacity",
            Self::SymbolSizeMismatch => "symbol length does not match the configured size",
            Self::EmptyWindow => "coding window size must be nonzero",
            Self::WindowCapacityExceeded => "coding window exceeds a capacity",
            Self::EquationCapacityExceeded => "invalid decoder equation capacity",
            Self::InvalidDensityThreshold => "density threshold must be in 0..=15",
            Self::RepairKeyMustBeZero => "full-density GF(2) repairs require repair key zero",
            Self::EmptyEncodingWindow => "cannot encode a repair without source symbols",
            Self::RepairWindowTooLarge => "repair window exceeds the decoder window",
            Self::WindowExpired => "symbol range has expired from the decoder window",
            Self::AmbiguousSequence => "sequence ordering is ambiguous",
            Self::EquationStoreFull => "decoder equation store is full",
            Self::ConflictingSource => "source symbol conflicts with known data",
            Self::InconsistentEquation => "repair equation contradicts known data",
        })
    }
}

impl core::error::Error for Error {}

/// The active parameters for an RLC encoder or decoder.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RlcConfig {
    symbol_size: usize,
    window_symbols: usize,
    field: Field,
}

/// A wrapping RFC 8681 Encoding Symbol ID (ESI).
///
/// This transparent newtype prevents symbol identifiers from being confused
/// with packet sequence numbers or ordinary counters.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct EncodingSymbolId(u32);

impl EncodingSymbolId {
    /// Creates an ESI from its RFC wire value.
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    /// Returns the RFC wire value.
    pub const fn get(self) -> u32 {
        self.0
    }

    /// Advances this ESI with RFC sequence-space wrapping.
    pub const fn wrapping_add(self, count: u32) -> Self {
        Self(self.0.wrapping_add(count))
    }

    /// Moves this ESI backward with RFC sequence-space wrapping.
    pub const fn wrapping_sub(self, count: u32) -> Self {
        Self(self.0.wrapping_sub(count))
    }
}

impl From<u32> for EncodingSymbolId {
    fn from(value: u32) -> Self {
        Self::new(value)
    }
}

impl From<EncodingSymbolId> for u32 {
    fn from(value: EncodingSymbolId) -> Self {
        value.get()
    }
}

impl fmt::Display for EncodingSymbolId {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(formatter)
    }
}

impl RlcConfig {
    /// Creates a configuration.
    pub const fn new(
        symbol_size: usize,
        window_symbols: usize,
        field: Field,
    ) -> Result<Self, Error> {
        if symbol_size == 0 {
            return Err(Error::EmptySymbol);
        }
        if symbol_size > u16::MAX as usize {
            return Err(Error::SymbolCapacityExceeded);
        }
        if window_symbols == 0 {
            return Err(Error::EmptyWindow);
        }
        if window_symbols > RFC_MAX_WINDOW_SYMBOLS {
            return Err(Error::WindowCapacityExceeded);
        }
        Ok(Self {
            symbol_size,
            window_symbols,
            field,
        })
    }

    /// Returns the active size of every source and repair symbol, in bytes.
    pub const fn symbol_size(self) -> usize {
        self.symbol_size
    }

    /// Returns the maximum number of symbols in the active coding window.
    pub const fn window_symbols(self) -> usize {
        self.window_symbols
    }

    /// Returns the configured finite field.
    pub const fn field(self) -> Field {
        self.field
    }
}

/// An RFC 8681 coding-coefficient density threshold in the range `0..=15`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DensityThreshold(u8);

impl DensityThreshold {
    /// Full density: every coding coefficient is nonzero.
    pub const FULL: Self = Self(15);

    /// Creates a checked threshold.
    pub const fn new(value: u8) -> Result<Self, Error> {
        if value <= 15 {
            Ok(Self(value))
        } else {
            Err(Error::InvalidDensityThreshold)
        }
    }

    /// Returns the wire value.
    pub const fn get(self) -> u8 {
        self.0
    }
}

impl TryFrom<u8> for DensityThreshold {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Self::new(value)
    }
}

impl From<DensityThreshold> for u8 {
    fn from(value: DensityThreshold) -> Self {
        value.get()
    }
}

/// Per-repair parameters that need not be fixed for the lifetime of an encoder.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RepairParameters {
    repair_key: u16,
    density: DensityThreshold,
}

impl RepairParameters {
    /// Creates repair parameters from a coefficient-generator seed and density.
    pub const fn new(repair_key: u16, density: DensityThreshold) -> Self {
        Self {
            repair_key,
            density,
        }
    }

    /// Returns the repair key.
    pub const fn repair_key(self) -> u16 {
        self.repair_key
    }

    /// Returns the density threshold.
    pub const fn density(self) -> DensityThreshold {
        self.density
    }
}

/// The result of submitting a source symbol to a decoder.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SourceStatus {
    /// This source symbol was not previously known.
    New,
    /// This exact source symbol was already known, either received or recovered.
    Duplicate,
}

/// The result of submitting a repair symbol to a decoder.
#[must_use]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DecodeReport {
    innovative: bool,
    recovered: usize,
}

/// The result of submitting a systematic source symbol to a decoder.
#[must_use]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SourceReport {
    status: SourceStatus,
    recovered: usize,
}

impl SourceReport {
    /// Returns whether the submitted source was new or a duplicate.
    pub const fn status(self) -> SourceStatus {
        self.status
    }

    /// Returns the number of other source symbols recovered while processing it.
    pub const fn recovered(self) -> usize {
        self.recovered
    }
}

impl DecodeReport {
    /// Returns whether the repair increased the decoder's equation rank.
    pub const fn is_innovative(self) -> bool {
        self.innovative
    }

    /// Returns the number of source symbols recovered while processing the repair.
    pub const fn recovered(self) -> usize {
        self.recovered
    }
}

/// Generates RFC 8681 coding coefficients into `output`.
///
/// The repair key initializes RFC 8682 TinyMT32. Each output byte is `0` or `1`
/// for [`Field::Gf2`] and is an arbitrary field element for [`Field::Gf256`].
pub fn generate_coding_coefficients(
    field: Field,
    repair_key: u16,
    density: DensityThreshold,
    output: &mut [u8],
) {
    if field == Field::Gf2 && density == DensityThreshold::FULL {
        output.fill(1);
        return;
    }

    let mut generator = TinyMt32::new(u32::from(repair_key));
    match field {
        Field::Gf2 => {
            for coefficient in output {
                *coefficient = u8::from(generator.next_u4() <= density.get());
            }
        }
        Field::Gf256 => {
            for coefficient in output {
                if density != DensityThreshold::FULL && generator.next_u4() > density.get() {
                    *coefficient = 0;
                    continue;
                }
                *coefficient = nonzero_byte(&mut generator);
            }
        }
    }
}

fn nonzero_byte(generator: &mut TinyMt32) -> u8 {
    loop {
        let value = generator.next_u8();
        if value != 0 {
            return value;
        }
    }
}

/// A systematic, bounded sliding-window RFC 8681 encoder.
///
/// `MAX_SYMBOL_SIZE` and `MAX_WINDOW_SYMBOLS` are hard memory capacities. The
/// active sizes come from [`RlcConfig`] and may be smaller.
pub struct Encoder<const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize> {
    config: RlcConfig,
    symbols: [[u8; MAX_SYMBOL_SIZE]; MAX_WINDOW_SYMBOLS],
    head: usize,
    len: usize,
    first_esi: EncodingSymbolId,
    next_esi: EncodingSymbolId,
}

impl<const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize>
    Encoder<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS>
{
    /// Creates an empty encoder whose first source symbol uses `initial_esi`.
    pub fn new(config: RlcConfig, initial_esi: EncodingSymbolId) -> Result<Self, Error> {
        validate_capacities::<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS>(config)?;
        Ok(Self {
            config,
            symbols: [[0; MAX_SYMBOL_SIZE]; MAX_WINDOW_SYMBOLS],
            head: 0,
            len: 0,
            first_esi: initial_esi,
            next_esi: initial_esi,
        })
    }

    /// Returns the active configuration.
    pub const fn config(&self) -> RlcConfig {
        self.config
    }

    /// Returns the number of source symbols currently retained.
    pub const fn window_len(&self) -> usize {
        self.len
    }

    /// Returns the ESI and length of the current encoding window.
    pub const fn window(&self) -> Option<(EncodingSymbolId, usize)> {
        if self.len == 0 {
            None
        } else {
            Some((self.first_esi, self.len))
        }
    }

    /// Returns the ESI that will be assigned by the next [`push_source`](Self::push_source).
    pub const fn next_esi(&self) -> EncodingSymbolId {
        self.next_esi
    }

    /// Copies a source symbol into the encoding window and returns its ESI.
    ///
    /// Once the active window is full, this evicts exactly one oldest symbol.
    pub fn push_source(&mut self, symbol: &[u8]) -> Result<EncodingSymbolId, Error> {
        self.check_symbol_size(symbol)?;
        let esi = self.next_esi;
        self.next_esi = self.next_esi.wrapping_add(1);

        let index = if self.len < self.config.window_symbols {
            let index = (self.head + self.len) % self.config.window_symbols;
            self.len += 1;
            index
        } else {
            let index = self.head;
            self.head = (self.head + 1) % self.config.window_symbols;
            self.first_esi = self.first_esi.wrapping_add(1);
            index
        };
        self.symbols[index][..self.config.symbol_size].copy_from_slice(symbol);
        Ok(esi)
    }

    /// Removes up to `count` oldest symbols from the coding window.
    ///
    /// This supports application-level validity expiry in addition to automatic
    /// eviction at the configured window size.
    pub fn discard_oldest(&mut self, count: usize) {
        let removed = count.min(self.len);
        if removed == 0 {
            return;
        }
        self.head = (self.head + removed) % self.config.window_symbols;
        self.len -= removed;
        self.first_esi = self.first_esi.wrapping_add(removed as u32);
        if self.len == 0 {
            self.first_esi = self.next_esi;
        }
    }

    /// Encodes one repair symbol for the current window.
    ///
    /// The returned value contains all metadata needed to regenerate the coding
    /// coefficients. The caller owns packetization and transport.
    pub fn encode_repair(
        &self,
        parameters: RepairParameters,
        output: &mut [u8],
    ) -> Result<RepairPayloadId, Error> {
        self.check_symbol_size(output)?;
        if self.len == 0 {
            return Err(Error::EmptyEncodingWindow);
        }
        if self.config.field == Field::Gf2
            && parameters.density == DensityThreshold::FULL
            && parameters.repair_key != 0
        {
            return Err(Error::RepairKeyMustBeZero);
        }

        output.fill(0);
        match self.config.field {
            Field::Gf2 if parameters.density == DensityThreshold::FULL => {
                for offset in 0..self.len {
                    self.add_source_to_repair(output, offset, 1);
                }
            }
            Field::Gf2 => {
                let mut generator = TinyMt32::new(u32::from(parameters.repair_key));
                for offset in 0..self.len {
                    let coefficient = u8::from(generator.next_u4() <= parameters.density.get());
                    if coefficient != 0 {
                        self.add_source_to_repair(output, offset, coefficient);
                    }
                }
            }
            Field::Gf256 => {
                let mut generator = TinyMt32::new(u32::from(parameters.repair_key));
                for offset in 0..self.len {
                    if parameters.density != DensityThreshold::FULL
                        && generator.next_u4() > parameters.density.get()
                    {
                        continue;
                    }
                    self.add_source_to_repair(output, offset, nonzero_byte(&mut generator));
                }
            }
        }

        RepairPayloadId::new(
            parameters.repair_key,
            parameters.density,
            self.len as u16,
            self.first_esi,
        )
        .map_err(|_| Error::WindowCapacityExceeded)
    }

    fn check_symbol_size(&self, symbol: &[u8]) -> Result<(), Error> {
        if symbol.len() == self.config.symbol_size {
            Ok(())
        } else {
            Err(Error::SymbolSizeMismatch)
        }
    }

    #[inline]
    fn add_source_to_repair(&self, output: &mut [u8], offset: usize, coefficient: u8) {
        let index = (self.head + offset) % self.config.window_symbols;
        add_scaled(
            self.config.field,
            output,
            coefficient,
            &self.symbols[index][..self.config.symbol_size],
        );
    }
}

#[derive(Clone, Copy)]
struct Row<const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize> {
    coefficients: [u8; MAX_WINDOW_SYMBOLS],
    data: [u8; MAX_SYMBOL_SIZE],
}

impl<const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize>
    Row<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS>
{
    const ZERO: Self = Self {
        coefficients: [0; MAX_WINDOW_SYMBOLS],
        data: [0; MAX_SYMBOL_SIZE],
    };
}

/// A bounded RFC 8681 sliding-window decoder.
///
/// The decoder maintains a reduced linear system without allocation. Source
/// symbols are addressable with [`symbol`](Self::symbol) while they remain in
/// the active window. Packet integrity must be checked before symbols are passed
/// here because erasure coding cannot authenticate corrupted input.
pub struct Decoder<
    const MAX_SYMBOL_SIZE: usize,
    const MAX_WINDOW_SYMBOLS: usize,
    const MAX_EQUATIONS: usize,
> {
    config: RlcConfig,
    equation_capacity: usize,
    base_esi: EncodingSymbolId,
    base_initialized: bool,
    span: usize,
    known: [bool; MAX_WINDOW_SYMBOLS],
    symbols: [[u8; MAX_SYMBOL_SIZE]; MAX_WINDOW_SYMBOLS],
    occupied: [bool; MAX_EQUATIONS],
    rows: [Row<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS>; MAX_EQUATIONS],
    rank: usize,
}

/// An iterator over received and recovered symbols retained by a decoder.
pub struct KnownSymbols<
    'a,
    const MAX_SYMBOL_SIZE: usize,
    const MAX_WINDOW_SYMBOLS: usize,
    const MAX_EQUATIONS: usize,
> {
    decoder: &'a Decoder<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS, MAX_EQUATIONS>,
    index: usize,
}

impl<'a, const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize, const MAX_EQUATIONS: usize>
    Iterator for KnownSymbols<'a, MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS, MAX_EQUATIONS>
{
    type Item = (EncodingSymbolId, &'a [u8]);

    fn next(&mut self) -> Option<Self::Item> {
        while self.index < self.decoder.span {
            let index = self.index;
            self.index += 1;
            if self.decoder.known[index] {
                return Some((
                    self.decoder.base_esi.wrapping_add(index as u32),
                    &self.decoder.symbols[index][..self.decoder.config.symbol_size],
                ));
            }
        }
        None
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (0, Some(self.decoder.span.saturating_sub(self.index)))
    }
}

impl<const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize, const MAX_EQUATIONS: usize>
    core::iter::FusedIterator
    for KnownSymbols<'_, MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS, MAX_EQUATIONS>
{
}

impl<const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize, const MAX_EQUATIONS: usize>
    Decoder<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS, MAX_EQUATIONS>
{
    /// Creates an empty decoder with a bounded active equation count.
    pub fn new(config: RlcConfig, equation_capacity: usize) -> Result<Self, Error> {
        validate_capacities::<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS>(config)?;
        if equation_capacity == 0 || equation_capacity > MAX_EQUATIONS {
            return Err(Error::EquationCapacityExceeded);
        }
        Ok(Self {
            config,
            equation_capacity,
            base_esi: EncodingSymbolId::new(0),
            base_initialized: false,
            span: 0,
            known: [false; MAX_WINDOW_SYMBOLS],
            symbols: [[0; MAX_SYMBOL_SIZE]; MAX_WINDOW_SYMBOLS],
            occupied: [false; MAX_EQUATIONS],
            rows: [Row::ZERO; MAX_EQUATIONS],
            rank: 0,
        })
    }

    /// Returns the active configuration.
    pub const fn config(&self) -> RlcConfig {
        self.config
    }

    /// Returns the current rank of unresolved repair equations.
    pub const fn rank(&self) -> usize {
        self.rank
    }

    /// Returns the ESI and total span of the retained decoding window.
    pub const fn window(&self) -> Option<(EncodingSymbolId, usize)> {
        if self.span == 0 {
            None
        } else {
            Some((self.base_esi, self.span))
        }
    }

    /// Discards decoder state older than `first_retained_esi`.
    ///
    /// Equations that still depend on an unknown expired symbol are discarded.
    /// This lets an integrating real-time protocol enforce its latency budget
    /// even when no newer source symbol arrives.
    pub fn discard_before(&mut self, first_retained_esi: EncodingSymbolId) -> Result<(), Error> {
        if self.span == 0 {
            if !self.base_initialized {
                self.base_esi = first_retained_esi;
                self.base_initialized = true;
                return Ok(());
            }
            let count = sequence_delta(first_retained_esi, self.base_esi)?;
            if count > 0 {
                self.base_esi = first_retained_esi;
            }
            return Ok(());
        }
        let count = sequence_delta(first_retained_esi, self.base_esi)?;
        if count > 0 {
            self.drop_front(count as usize);
        }
        Ok(())
    }

    /// Returns a received or recovered symbol while it remains in the window.
    pub fn symbol(&self, esi: EncodingSymbolId) -> Option<&[u8]> {
        let offset = sequence_delta(esi, self.base_esi).ok()?;
        if offset < 0 {
            return None;
        }
        let index = offset as usize;
        if index >= self.span || !self.known[index] {
            return None;
        }
        Some(&self.symbols[index][..self.config.symbol_size])
    }

    /// Iterates over all known symbols in ascending wrapping ESI order.
    pub fn known_symbols(
        &self,
    ) -> KnownSymbols<'_, MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS, MAX_EQUATIONS> {
        KnownSymbols {
            decoder: self,
            index: 0,
        }
    }

    /// Submits a systematic source symbol.
    pub fn receive_source(
        &mut self,
        esi: EncodingSymbolId,
        symbol: &[u8],
    ) -> Result<SourceReport, Error> {
        self.check_symbol_size(symbol)?;
        let index = self.ensure_range(esi, 1)?;
        if self.known[index] {
            if self.symbols[index][..self.config.symbol_size] == *symbol {
                return Ok(SourceReport {
                    status: SourceStatus::Duplicate,
                    recovered: 0,
                });
            }
            return Err(Error::ConflictingSource);
        }

        self.store_known(index, symbol);
        self.substitute_known(index)?;
        let recovered = self.recover_singletons()?;
        Ok(SourceReport {
            status: SourceStatus::New,
            recovered,
        })
    }

    /// Submits one repair symbol and returns its contribution to decoding.
    pub fn receive_repair(
        &mut self,
        id: RepairPayloadId,
        symbol: &[u8],
    ) -> Result<DecodeReport, Error> {
        self.check_symbol_size(symbol)?;
        let source_count = usize::from(id.source_symbols());
        if source_count > self.config.window_symbols {
            return Err(Error::RepairWindowTooLarge);
        }
        let start = self.ensure_range(id.first_source_esi(), source_count)?;

        let mut row = Row::ZERO;
        generate_coding_coefficients(
            self.config.field,
            id.repair_key(),
            id.density(),
            &mut row.coefficients[start..start + source_count],
        );
        row.data[..self.config.symbol_size].copy_from_slice(symbol);

        for index in start..start + source_count {
            let coefficient = row.coefficients[index];
            if coefficient != 0 && self.known[index] {
                add_scaled(
                    self.config.field,
                    &mut row.data[..self.config.symbol_size],
                    coefficient,
                    &self.symbols[index][..self.config.symbol_size],
                );
                row.coefficients[index] = 0;
            }
        }

        for row_index in 0..self.equation_capacity {
            if !self.occupied[row_index] {
                continue;
            }
            let Some(pivot) = first_nonzero(&self.rows[row_index].coefficients[..self.span]) else {
                continue;
            };
            let factor = row.coefficients[pivot];
            if factor != 0 {
                row_add_scaled(
                    self.config.field,
                    &mut row,
                    factor,
                    &self.rows[row_index],
                    self.span,
                    self.config.symbol_size,
                );
            }
        }

        let Some(pivot) = first_nonzero(&row.coefficients[..self.span]) else {
            if row.data[..self.config.symbol_size]
                .iter()
                .any(|byte| *byte != 0)
            {
                return Err(Error::InconsistentEquation);
            }
            return Ok(DecodeReport {
                innovative: false,
                recovered: 0,
            });
        };

        let Some(slot) = self.occupied[..self.equation_capacity]
            .iter()
            .position(|occupied| !occupied)
        else {
            return Err(Error::EquationStoreFull);
        };

        let inverse = self.config.field.inv(row.coefficients[pivot]);
        row_scale(
            self.config.field,
            &mut row,
            inverse,
            self.span,
            self.config.symbol_size,
        );

        for row_index in 0..self.equation_capacity {
            if !self.occupied[row_index] {
                continue;
            }
            let factor = self.rows[row_index].coefficients[pivot];
            if factor != 0 {
                row_add_scaled(
                    self.config.field,
                    &mut self.rows[row_index],
                    factor,
                    &row,
                    self.span,
                    self.config.symbol_size,
                );
            }
        }

        self.rows[slot] = row;
        self.occupied[slot] = true;
        self.rank += 1;
        let recovered = self.recover_singletons()?;
        Ok(DecodeReport {
            innovative: true,
            recovered,
        })
    }

    fn check_symbol_size(&self, symbol: &[u8]) -> Result<(), Error> {
        if symbol.len() == self.config.symbol_size {
            Ok(())
        } else {
            Err(Error::SymbolSizeMismatch)
        }
    }

    fn ensure_range(&mut self, first_esi: EncodingSymbolId, count: usize) -> Result<usize, Error> {
        debug_assert!(count > 0);
        if self.span == 0 {
            if !self.base_initialized {
                self.base_esi = first_esi;
                self.base_initialized = true;
                self.span = count;
                return Ok(0);
            }

            let start = sequence_delta(first_esi, self.base_esi)?;
            if start < 0 {
                return Err(Error::WindowExpired);
            }
            let end = start as usize + count;
            if end <= self.config.window_symbols {
                self.extend_to(end);
                return Ok(start as usize);
            }
            self.base_esi = first_esi
                .wrapping_add(count as u32)
                .wrapping_sub(self.config.window_symbols as u32);
            self.span = self.config.window_symbols;
            return Ok(self.config.window_symbols - count);
        }

        let start = sequence_delta(first_esi, self.base_esi)?;
        let end = start + count as i64 - 1;
        let current_end = self.span as i64 - 1;
        let union_start = start.min(0);
        let union_end = end.max(current_end);
        let union_len = union_end - union_start + 1;
        let capacity = self.config.window_symbols as i64;

        if union_len <= capacity {
            if union_start < 0 {
                self.prepend((-union_start) as usize);
            }
            let wanted_span = union_len as usize;
            self.extend_to(wanted_span);
            return sequence_delta(first_esi, self.base_esi).map(|offset| offset as usize);
        }

        let new_base_offset = union_end - capacity + 1;
        if new_base_offset <= 0 || start < new_base_offset {
            return Err(Error::WindowExpired);
        }
        self.drop_front(new_base_offset as usize);
        let new_end = sequence_delta(first_esi.wrapping_add((count - 1) as u32), self.base_esi)?;
        self.extend_to(new_end as usize + 1);
        sequence_delta(first_esi, self.base_esi).map(|offset| offset as usize)
    }

    fn prepend(&mut self, count: usize) {
        debug_assert!(self.span + count <= self.config.window_symbols);
        for index in (0..self.span).rev() {
            self.known[index + count] = self.known[index];
            self.symbols[index + count] = self.symbols[index];
        }
        for index in 0..count {
            self.known[index] = false;
            self.symbols[index].fill(0);
        }
        for row_index in 0..self.equation_capacity {
            if !self.occupied[row_index] {
                continue;
            }
            for index in (0..self.span).rev() {
                self.rows[row_index].coefficients[index + count] =
                    self.rows[row_index].coefficients[index];
            }
            self.rows[row_index].coefficients[..count].fill(0);
        }
        self.base_esi = self.base_esi.wrapping_sub(count as u32);
        self.span += count;
    }

    fn extend_to(&mut self, new_span: usize) {
        debug_assert!(new_span <= self.config.window_symbols);
        for index in self.span..new_span {
            self.known[index] = false;
            self.symbols[index].fill(0);
        }
        self.span = self.span.max(new_span);
    }

    fn drop_front(&mut self, count: usize) {
        if count >= self.span {
            self.known[..self.span].fill(false);
            self.clear_equations();
            self.base_esi = self.base_esi.wrapping_add(count as u32);
            self.span = 0;
            return;
        }

        let old_span = self.span;
        let retained = old_span - count;
        for row_index in 0..self.equation_capacity {
            if !self.occupied[row_index] {
                continue;
            }
            if self.rows[row_index].coefficients[..count]
                .iter()
                .any(|coefficient| *coefficient != 0)
            {
                self.occupied[row_index] = false;
                self.rank -= 1;
                continue;
            }
            self.rows[row_index]
                .coefficients
                .copy_within(count..old_span, 0);
            self.rows[row_index].coefficients[retained..old_span].fill(0);
        }
        self.known.copy_within(count..old_span, 0);
        self.symbols.copy_within(count..old_span, 0);
        self.known[retained..old_span].fill(false);
        for symbol in &mut self.symbols[retained..old_span] {
            symbol.fill(0);
        }
        self.base_esi = self.base_esi.wrapping_add(count as u32);
        self.span = retained;
    }

    fn clear_equations(&mut self) {
        self.occupied[..self.equation_capacity].fill(false);
        self.rank = 0;
    }

    fn store_known(&mut self, index: usize, symbol: &[u8]) {
        self.symbols[index][..self.config.symbol_size].copy_from_slice(symbol);
        self.known[index] = true;
    }

    fn substitute_known(&mut self, index: usize) -> Result<(), Error> {
        for row_index in 0..self.equation_capacity {
            if !self.occupied[row_index] {
                continue;
            }
            let coefficient = self.rows[row_index].coefficients[index];
            if coefficient == 0 {
                continue;
            }
            add_scaled(
                self.config.field,
                &mut self.rows[row_index].data[..self.config.symbol_size],
                coefficient,
                &self.symbols[index][..self.config.symbol_size],
            );
            self.rows[row_index].coefficients[index] = 0;
        }
        self.remove_zero_rows()
    }

    fn remove_zero_rows(&mut self) -> Result<(), Error> {
        for row_index in 0..self.equation_capacity {
            if !self.occupied[row_index]
                || self.rows[row_index].coefficients[..self.span]
                    .iter()
                    .any(|coefficient| *coefficient != 0)
            {
                continue;
            }
            if self.rows[row_index].data[..self.config.symbol_size]
                .iter()
                .any(|byte| *byte != 0)
            {
                return Err(Error::InconsistentEquation);
            }
            self.occupied[row_index] = false;
            self.rank -= 1;
        }
        Ok(())
    }

    fn recover_singletons(&mut self) -> Result<usize, Error> {
        let mut recovered = 0;
        loop {
            let mut singleton = None;
            for row_index in 0..self.equation_capacity {
                if !self.occupied[row_index] {
                    continue;
                }
                let mut only = None;
                let mut multiple = false;
                for (index, coefficient) in self.rows[row_index].coefficients[..self.span]
                    .iter()
                    .copied()
                    .enumerate()
                {
                    if coefficient == 0 {
                        continue;
                    }
                    if only.is_some() {
                        multiple = true;
                        break;
                    }
                    only = Some(index);
                }
                if !multiple && let Some(symbol_index) = only {
                    singleton = Some((row_index, symbol_index));
                    break;
                }
            }

            let Some((row_index, symbol_index)) = singleton else {
                break;
            };
            let coefficient = self.rows[row_index].coefficients[symbol_index];
            let inverse = self.config.field.inv(coefficient);
            if self.known[symbol_index] {
                let matches = if inverse == 1 {
                    self.symbols[symbol_index][..self.config.symbol_size]
                        == self.rows[row_index].data[..self.config.symbol_size]
                } else {
                    self.symbols[symbol_index][..self.config.symbol_size]
                        .iter()
                        .zip(&self.rows[row_index].data[..self.config.symbol_size])
                        .all(|(known, encoded)| *known == self.config.field.mul(*encoded, inverse))
                };
                if !matches {
                    return Err(Error::ConflictingSource);
                }
            } else {
                if inverse == 1 {
                    self.symbols[symbol_index][..self.config.symbol_size]
                        .copy_from_slice(&self.rows[row_index].data[..self.config.symbol_size]);
                } else {
                    for (decoded, encoded) in self.symbols[symbol_index][..self.config.symbol_size]
                        .iter_mut()
                        .zip(&self.rows[row_index].data[..self.config.symbol_size])
                    {
                        *decoded = self.config.field.mul(*encoded, inverse);
                    }
                }
                self.known[symbol_index] = true;
                recovered += 1;
            }
            self.occupied[row_index] = false;
            self.rank -= 1;
            self.substitute_known(symbol_index)?;
        }
        Ok(recovered)
    }
}

fn validate_capacities<const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize>(
    config: RlcConfig,
) -> Result<(), Error> {
    if config.symbol_size > MAX_SYMBOL_SIZE {
        return Err(Error::SymbolCapacityExceeded);
    }
    if config.window_symbols > MAX_WINDOW_SYMBOLS {
        return Err(Error::WindowCapacityExceeded);
    }
    Ok(())
}

fn sequence_delta(value: EncodingSymbolId, base: EncodingSymbolId) -> Result<i64, Error> {
    let distance = value.get().wrapping_sub(base.get());
    if distance == 0x8000_0000 {
        return Err(Error::AmbiguousSequence);
    }
    Ok(i64::from(distance as i32))
}

fn add_scaled(field: Field, target: &mut [u8], coefficient: u8, source: &[u8]) {
    debug_assert_eq!(target.len(), source.len());
    match (field, coefficient) {
        (_, 0) => {}
        (Field::Gf2, _) | (Field::Gf256, 1) => {
            for (target, source) in target.iter_mut().zip(source) {
                *target ^= source;
            }
        }
        (Field::Gf256, _) => {
            for (target, source) in target.iter_mut().zip(source) {
                *target ^= field.mul(coefficient, *source);
            }
        }
    }
}

fn first_nonzero(values: &[u8]) -> Option<usize> {
    values.iter().position(|value| *value != 0)
}

fn row_add_scaled<const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize>(
    field: Field,
    target: &mut Row<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS>,
    coefficient: u8,
    source: &Row<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS>,
    span: usize,
    symbol_size: usize,
) {
    for (target, source) in target.coefficients[..span]
        .iter_mut()
        .zip(&source.coefficients[..span])
    {
        *target ^= field.mul(coefficient, *source);
    }
    add_scaled(
        field,
        &mut target.data[..symbol_size],
        coefficient,
        &source.data[..symbol_size],
    );
}

fn row_scale<const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize>(
    field: Field,
    row: &mut Row<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS>,
    coefficient: u8,
    span: usize,
    symbol_size: usize,
) {
    if coefficient == 1 {
        return;
    }
    for value in &mut row.coefficients[..span] {
        *value = field.mul(*value, coefficient);
    }
    for value in &mut row.data[..symbol_size] {
        *value = field.mul(*value, coefficient);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SYMBOL_CAPACITY: usize = 16;
    const WINDOW_CAPACITY: usize = 8;
    const EQUATION_CAPACITY: usize = 8;

    fn config(field: Field) -> RlcConfig {
        RlcConfig::new(4, WINDOW_CAPACITY, field).unwrap()
    }

    const fn esi(value: u32) -> EncodingSymbolId {
        EncodingSymbolId::new(value)
    }

    #[test]
    fn coefficient_vectors_match_rfc_small_integer_outputs() {
        let mut coefficients = [0; 10];
        generate_coding_coefficients(Field::Gf256, 1, DensityThreshold::FULL, &mut coefficients);
        assert_eq!(
            coefficients,
            [37, 225, 177, 176, 21, 246, 54, 139, 168, 237]
        );

        generate_coding_coefficients(
            Field::Gf2,
            1,
            DensityThreshold::new(7).unwrap(),
            &mut coefficients,
        );
        assert_eq!(coefficients, [1, 1, 1, 1, 1, 1, 1, 0, 0, 0]);

        generate_coding_coefficients(
            Field::Gf256,
            1,
            DensityThreshold::new(7).unwrap(),
            &mut coefficients,
        );
        assert_eq!(coefficients, [225, 176, 246, 139, 0, 0, 187, 0, 0, 0]);
    }

    #[test]
    fn encoder_is_systematic_and_slides_across_esi_wrap() {
        let mut encoder = Encoder::<SYMBOL_CAPACITY, 3>::new(
            RlcConfig::new(4, 3, Field::Gf2).unwrap(),
            esi(u32::MAX - 1),
        )
        .unwrap();
        assert_eq!(encoder.push_source(&[1, 2, 3, 4]), Ok(esi(u32::MAX - 1)));
        assert_eq!(encoder.push_source(&[5, 6, 7, 8]), Ok(esi(u32::MAX)));
        assert_eq!(encoder.push_source(&[9, 10, 11, 12]), Ok(esi(0)));
        assert_eq!(encoder.push_source(&[13, 14, 15, 16]), Ok(esi(1)));

        let mut repair = [0; 4];
        let id = encoder
            .encode_repair(
                RepairParameters::new(0, DensityThreshold::FULL),
                &mut repair,
            )
            .unwrap();
        assert_eq!(id.first_source_esi(), esi(u32::MAX));
        assert_eq!(id.source_symbols(), 3);
        assert_eq!(repair, [1, 2, 3, 20]);
    }

    #[test]
    fn decoder_recovers_one_missing_binary_symbol() {
        let config = config(Field::Gf2);
        let mut encoder =
            Encoder::<SYMBOL_CAPACITY, WINDOW_CAPACITY>::new(config, esi(10)).unwrap();
        let mut decoder = Decoder::<SYMBOL_CAPACITY, WINDOW_CAPACITY, EQUATION_CAPACITY>::new(
            config,
            EQUATION_CAPACITY,
        )
        .unwrap();

        let symbols = [[1, 2, 3, 4], [9, 8, 7, 6], [3, 1, 4, 1]];
        for symbol in &symbols {
            encoder.push_source(symbol).unwrap();
        }
        let _ = decoder.receive_source(esi(10), &symbols[0]).unwrap();
        let _ = decoder.receive_source(esi(12), &symbols[2]).unwrap();

        let mut repair = [0; 4];
        let id = encoder
            .encode_repair(
                RepairParameters::new(0, DensityThreshold::FULL),
                &mut repair,
            )
            .unwrap();
        let report = decoder.receive_repair(id, &repair).unwrap();
        assert!(report.is_innovative());
        assert_eq!(report.recovered(), 1);
        assert_eq!(decoder.symbol(esi(11)), Some(symbols[1].as_slice()));
        assert!(!decoder.receive_repair(id, &repair).unwrap().is_innovative());
    }

    #[test]
    fn decoder_reduces_multiple_gf256_equations() {
        let config = config(Field::Gf256);
        let mut encoder =
            Encoder::<SYMBOL_CAPACITY, WINDOW_CAPACITY>::new(config, esi(50)).unwrap();
        let mut decoder = Decoder::<SYMBOL_CAPACITY, WINDOW_CAPACITY, EQUATION_CAPACITY>::new(
            config,
            EQUATION_CAPACITY,
        )
        .unwrap();
        let symbols = [[0, 1, 2, 3], [4, 5, 6, 7], [8, 9, 10, 11], [12, 13, 14, 15]];
        for symbol in &symbols {
            encoder.push_source(symbol).unwrap();
        }
        let _ = decoder.receive_source(esi(50), &symbols[0]).unwrap();
        let _ = decoder.receive_source(esi(53), &symbols[3]).unwrap();

        let mut repair_a = [0; 4];
        let mut repair_b = [0; 4];
        let id_a = encoder
            .encode_repair(
                RepairParameters::new(41, DensityThreshold::FULL),
                &mut repair_a,
            )
            .unwrap();
        let id_b = encoder
            .encode_repair(
                RepairParameters::new(42, DensityThreshold::FULL),
                &mut repair_b,
            )
            .unwrap();

        let _ = decoder.receive_repair(id_b, &repair_b).unwrap();
        let report = decoder.receive_repair(id_a, &repair_a).unwrap();
        assert_eq!(report.recovered(), 2);
        assert_eq!(decoder.symbol(esi(51)), Some(symbols[1].as_slice()));
        assert_eq!(decoder.symbol(esi(52)), Some(symbols[2].as_slice()));
        assert_eq!(decoder.rank(), 0);
    }

    #[test]
    fn decoder_accepts_reorder_duplicates_and_wrap() {
        let config = RlcConfig::new(4, 5, Field::Gf256).unwrap();
        let mut decoder = Decoder::<8, 5, 5>::new(config, 5).unwrap();
        let before_wrap = [1, 2, 3, 4];
        let after_wrap = [5, 6, 7, 8];

        assert_eq!(
            decoder
                .receive_source(esi(0), &after_wrap)
                .map(SourceReport::status),
            Ok(SourceStatus::New),
        );
        assert_eq!(
            decoder
                .receive_source(esi(u32::MAX), &before_wrap)
                .map(SourceReport::status),
            Ok(SourceStatus::New),
        );
        assert_eq!(
            decoder
                .receive_source(esi(0), &after_wrap)
                .map(SourceReport::status),
            Ok(SourceStatus::Duplicate),
        );
        assert_eq!(decoder.symbol(esi(u32::MAX)), Some(before_wrap.as_slice()));
        assert_eq!(decoder.symbol(esi(0)), Some(after_wrap.as_slice()));
    }

    #[test]
    fn repair_can_arrive_before_systematic_symbols() {
        let config = RlcConfig::new(4, 4, Field::Gf256).unwrap();
        let mut encoder = Encoder::<4, 4>::new(config, esi(100)).unwrap();
        let mut decoder = Decoder::<4, 4, 4>::new(config, 4).unwrap();
        let symbols = [[1, 3, 3, 7], [2, 4, 6, 8], [5, 5, 5, 5]];
        for symbol in &symbols {
            encoder.push_source(symbol).unwrap();
        }
        assert_eq!(encoder.window(), Some((esi(100), 3)));

        let mut repair = [0; 4];
        let id = encoder
            .encode_repair(
                RepairParameters::new(99, DensityThreshold::FULL),
                &mut repair,
            )
            .unwrap();
        assert_eq!(decoder.receive_repair(id, &repair).unwrap().recovered(), 0);
        let _ = decoder.receive_source(esi(102), &symbols[2]).unwrap();
        let source_report = decoder.receive_source(esi(100), &symbols[0]).unwrap();
        assert_eq!(source_report.recovered(), 1);
        assert_eq!(decoder.symbol(esi(101)), Some(symbols[1].as_slice()));

        decoder.discard_before(esi(102)).unwrap();
        assert_eq!(decoder.window(), Some((esi(102), 1)));
        assert_eq!(decoder.symbol(esi(100)), None);

        decoder.discard_before(esi(200)).unwrap();
        assert_eq!(decoder.window(), None);
        assert_eq!(
            decoder.receive_source(esi(199), &[0; 4]),
            Err(Error::WindowExpired)
        );
        assert_eq!(
            decoder
                .receive_source(esi(200), &[0; 4])
                .map(SourceReport::status),
            Ok(SourceStatus::New),
        );
        assert!(decoder.known_symbols().map(|(id, _)| id).eq([esi(200)]));
    }

    #[test]
    fn validates_sizes_capacities_and_binary_key_rule() {
        assert_eq!(RlcConfig::new(0, 1, Field::Gf2), Err(Error::EmptySymbol));
        assert_eq!(
            RlcConfig::new(1, 4096, Field::Gf2),
            Err(Error::WindowCapacityExceeded)
        );
        let config = RlcConfig::new(4, 4, Field::Gf2).unwrap();
        assert!(matches!(
            Encoder::<3, 4>::new(config, esi(0)),
            Err(Error::SymbolCapacityExceeded)
        ));
        assert!(matches!(
            Decoder::<4, 4, 1>::new(config, 2),
            Err(Error::EquationCapacityExceeded)
        ));

        let mut encoder = Encoder::<4, 4>::new(config, esi(0)).unwrap();
        assert_eq!(encoder.push_source(&[1, 2]), Err(Error::SymbolSizeMismatch));
        encoder.push_source(&[1, 2, 3, 4]).unwrap();
        assert_eq!(
            encoder.encode_repair(
                RepairParameters::new(1, DensityThreshold::FULL),
                &mut [0; 4],
            ),
            Err(Error::RepairKeyMustBeZero)
        );
    }

    #[test]
    fn equation_capacity_is_a_hard_bound() {
        let config = RlcConfig::new(1, 3, Field::Gf256).unwrap();
        let mut encoder = Encoder::<1, 3>::new(config, esi(0)).unwrap();
        let mut decoder = Decoder::<1, 3, 1>::new(config, 1).unwrap();
        for byte in 1..=3 {
            encoder.push_source(&[byte]).unwrap();
        }

        let mut repair = [0];
        let first = encoder
            .encode_repair(
                RepairParameters::new(1, DensityThreshold::FULL),
                &mut repair,
            )
            .unwrap();
        let _ = decoder.receive_repair(first, &repair).unwrap();
        let second = encoder
            .encode_repair(
                RepairParameters::new(2, DensityThreshold::FULL),
                &mut repair,
            )
            .unwrap();
        assert_eq!(
            decoder.receive_repair(second, &repair),
            Err(Error::EquationStoreFull)
        );
    }

    #[test]
    fn old_ranges_and_conflicting_duplicates_are_rejected() {
        let config = RlcConfig::new(1, 3, Field::Gf256).unwrap();
        let mut decoder = Decoder::<1, 3, 3>::new(config, 3).unwrap();
        let _ = decoder.receive_source(esi(10), &[10]).unwrap();
        let _ = decoder.receive_source(esi(11), &[11]).unwrap();
        let _ = decoder.receive_source(esi(12), &[12]).unwrap();
        let _ = decoder.receive_source(esi(13), &[13]).unwrap();

        assert_eq!(
            decoder.receive_source(esi(10), &[10]),
            Err(Error::WindowExpired),
        );
        assert_eq!(
            decoder.receive_source(esi(13), &[99]),
            Err(Error::ConflictingSource)
        );
    }
}
