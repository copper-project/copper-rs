use cu_fec::{
    DensityThreshold, EncodingSymbolId, Field, RepairParameters, RepairPayloadId, RlcConfig,
    RlcDecoder, RlcEncoder,
};

const SYMBOL_SIZE: usize = 32;
const WINDOW_SIZE: usize = 24;
const EQUATIONS: usize = 24;
const BATCH: usize = 12;

#[derive(Clone)]
struct RepairPacket {
    id: [u8; 8],
    symbol: [u8; SYMBOL_SIZE],
}

fn source_symbol(esi: EncodingSymbolId) -> [u8; SYMBOL_SIZE] {
    let mut state = esi.get() ^ 0xa5a5_5a5a;
    let mut symbol = [0; SYMBOL_SIZE];
    for byte in &mut symbol {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        *byte = state as u8;
    }
    symbol
}

fn run_burst_loss_scenario(field: Field) {
    let config = RlcConfig::new(SYMBOL_SIZE, WINDOW_SIZE, field).unwrap();
    let mut encoder =
        RlcEncoder::<SYMBOL_SIZE, WINDOW_SIZE>::new(config, EncodingSymbolId::new(u32::MAX - 47))
            .unwrap();
    let mut decoder =
        RlcDecoder::<SYMBOL_SIZE, WINDOW_SIZE, EQUATIONS>::new(config, EQUATIONS).unwrap();
    let density = if field == Field::Gf2 {
        DensityThreshold::new(7).unwrap()
    } else {
        DensityThreshold::FULL
    };

    for batch in 0..8_u16 {
        let mut received = Vec::new();
        let mut lost = Vec::new();
        for offset in 0..BATCH {
            let esi = encoder.next_esi();
            let symbol = source_symbol(esi);
            assert_eq!(encoder.push_source(&symbol), Ok(esi));
            if (3..6).contains(&offset) {
                lost.push((esi, symbol));
            } else {
                received.push((esi, symbol));
            }
        }

        // Reorder all systematic packets inside the bounded receiver window.
        for (esi, symbol) in received.iter().rev() {
            let _ = decoder.receive_source(*esi, symbol).unwrap();
        }

        let mut repairs = Vec::new();
        for offset in 0..10_u16 {
            let repair_key = if field == Field::Gf2 {
                batch.wrapping_mul(31).wrapping_add(offset)
            } else {
                batch.wrapping_mul(31).wrapping_add(offset).wrapping_add(1)
            };
            let mut symbol = [0; SYMBOL_SIZE];
            let id = encoder
                .encode_repair(RepairParameters::new(repair_key, density), &mut symbol)
                .unwrap();
            repairs.push(RepairPacket {
                id: id.to_bytes(),
                symbol,
            });
        }

        // Deliver repair packets in reverse order and duplicate every third packet.
        for (index, packet) in repairs.iter().rev().enumerate() {
            let id = RepairPayloadId::from_bytes(packet.id).unwrap();
            let _ = decoder.receive_repair(id, &packet.symbol).unwrap();
            if index.is_multiple_of(3) {
                let _ = decoder.receive_repair(id, &packet.symbol).unwrap();
            }
        }

        for (esi, expected) in lost {
            assert_eq!(decoder.symbol(esi), Some(expected.as_slice()), "ESI {esi}");
        }
    }
}

#[test]
fn gf256_recovers_reordered_bursts_across_sequence_wrap() {
    run_burst_loss_scenario(Field::Gf256);
}

#[test]
fn gf2_recovers_reordered_bursts_across_sequence_wrap() {
    run_burst_loss_scenario(Field::Gf2);
}
