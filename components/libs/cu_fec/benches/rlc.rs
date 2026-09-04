use std::{
    hint::black_box,
    mem::size_of,
    time::{Duration, Instant},
};

use cu_fec::{
    DensityThreshold, EncodingSymbolId, Field, RepairParameters, RlcConfig, RlcDecoder, RlcEncoder,
};

const SYMBOL_SIZE: usize = 256;
const WINDOW_SIZE: usize = 32;
const EQUATIONS: usize = 32;
const SAMPLE_TIME: Duration = Duration::from_millis(300);

fn main() {
    println!(
        "bounded state: encoder={} bytes decoder={} bytes",
        size_of::<RlcEncoder<SYMBOL_SIZE, WINDOW_SIZE>>(),
        size_of::<RlcDecoder<SYMBOL_SIZE, WINDOW_SIZE, EQUATIONS>>()
    );
    benchmark_encode(Field::Gf2, DensityThreshold::FULL, "encode/gf2/full");
    benchmark_encode(Field::Gf256, DensityThreshold::FULL, "encode/gf256/full");
    benchmark_single_loss_recovery();
}

fn benchmark_encode(field: Field, density: DensityThreshold, name: &str) {
    let config = RlcConfig::new(SYMBOL_SIZE, WINDOW_SIZE, field).unwrap();
    let mut encoder =
        RlcEncoder::<SYMBOL_SIZE, WINDOW_SIZE>::new(config, EncodingSymbolId::new(0)).unwrap();
    for esi in 0..WINDOW_SIZE {
        let mut symbol = [0_u8; SYMBOL_SIZE];
        fill_symbol(esi as u32, &mut symbol);
        encoder.push_source(black_box(&symbol)).unwrap();
    }

    let mut output = [0_u8; SYMBOL_SIZE];
    let mut repair_key = u16::from(field == Field::Gf256);
    measure(name, || {
        let parameters = RepairParameters::new(repair_key, density);
        let id = encoder
            .encode_repair(parameters, black_box(&mut output))
            .unwrap();
        black_box((id, &output));
        if field == Field::Gf256 {
            repair_key = repair_key.wrapping_add(1);
        }
    });
}

fn benchmark_single_loss_recovery() {
    let config = RlcConfig::new(SYMBOL_SIZE, WINDOW_SIZE, Field::Gf256).unwrap();
    let mut encoder =
        RlcEncoder::<SYMBOL_SIZE, WINDOW_SIZE>::new(config, EncodingSymbolId::new(0)).unwrap();
    let mut sources = [[0_u8; SYMBOL_SIZE]; WINDOW_SIZE];
    for (esi, symbol) in sources.iter_mut().enumerate() {
        fill_symbol(esi as u32, symbol);
        encoder.push_source(symbol).unwrap();
    }
    let mut repair = [0_u8; SYMBOL_SIZE];
    let id = encoder
        .encode_repair(
            RepairParameters::new(1, DensityThreshold::FULL),
            &mut repair,
        )
        .unwrap();

    measure("decode/gf256/one-loss-window", || {
        let mut decoder =
            RlcDecoder::<SYMBOL_SIZE, WINDOW_SIZE, EQUATIONS>::new(config, EQUATIONS).unwrap();
        for (esi, symbol) in sources.iter().enumerate().skip(1) {
            let _ = decoder
                .receive_source(EncodingSymbolId::new(esi as u32), black_box(symbol))
                .unwrap();
        }
        let report = decoder.receive_repair(id, black_box(&repair)).unwrap();
        debug_assert_eq!(report.recovered(), 1);
        black_box(decoder);
    });
}

fn measure(name: &str, mut operation: impl FnMut()) {
    for _ in 0..16 {
        operation();
    }
    let start = Instant::now();
    let mut iterations = 0_u64;
    while start.elapsed() < SAMPLE_TIME {
        operation();
        iterations += 1;
    }
    let nanoseconds = start.elapsed().as_nanos() as f64 / iterations as f64;
    println!("{name}: {nanoseconds:.1} ns/op ({iterations} samples)");
}

fn fill_symbol(seed: u32, output: &mut [u8]) {
    let mut state = seed ^ 0xa5a5_5a5a;
    for byte in output {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        *byte = state as u8;
    }
}
