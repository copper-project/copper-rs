#![cfg(feature = "sim")]

use std::collections::HashMap;
use std::io::{self, Error, ErrorKind};
use std::path::PathBuf;
use std::thread;
use std::time::Duration;

use evdev::{AbsoluteAxisCode, AttributeSetRef, Device, EventSummary, EventType, KeyCode};

/// Snapshot of all RC inputs normalized to [-1.0, 1.0].
#[derive(Debug, Clone)]
pub struct RcFrame {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
    pub knob_sa: f32,
    pub knob_sb: f32,
    pub knob_sc: f32,
    pub aux: Vec<AuxChannel>,
    pub switches: Vec<SwitchState>,
}

/// AUX channel with a stable name.
#[derive(Debug, Clone)]
pub struct AuxChannel {
    pub name: String,
    pub value: f32,
}

/// Switch/button state with a stable name.
#[derive(Debug, Clone)]
pub struct SwitchState {
    pub name: String,
    pub on: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum RcAxis {
    Roll,
    Pitch,
    Yaw,
    Throttle,
    KnobSa,
    KnobSb,
    KnobSc,
    Aux(u8),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum AxisScale {
    NegOneToOne,
    ZeroToOne,
}

/// Reader that discovers a joystick-like device and streams normalized values.
pub struct RcJoystick {
    device: Device,
    axis_map: HashMap<u16, (RcAxis, AxisScale)>,
    switch_map: HashMap<u16, usize>,
    state: RcFrame,
}

impl RcJoystick {
    /// Create a joystick reader. When `preferred_name` is provided it will match the
    /// device name (case insensitive). Otherwise the best joystick-like device is picked.
    pub fn open(preferred_name: Option<&str>) -> io::Result<Self> {
        let (path, device) = discover_device(preferred_name)?;

        let axis_map = build_axis_map(&device);
        if axis_map.is_empty() {
            return Err(Error::new(
                ErrorKind::NotFound,
                "joystick has no absolute axes",
            ));
        }

        let aux_count = axis_map
            .values()
            .filter(|a| matches!(a.0, RcAxis::Aux(_)))
            .count();
        let mut aux = Vec::with_capacity(aux_count);
        for i in 0..aux_count {
            aux.push(AuxChannel {
                name: format!("aux{}", i + 1),
                value: 0.0,
            });
        }

        let switches = build_switches(&device);
        let state = RcFrame {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 0.0,
            knob_sa: 0.0,
            knob_sb: 0.0,
            knob_sc: 0.0,
            aux,
            switches,
        };
        let switch_map = build_switch_map(&device, &state.switches);

        let mut joystick = Self {
            device,
            axis_map,
            switch_map,
            state,
        };

        prime_axes(&joystick.device, &joystick.axis_map, &mut joystick.state);
        prime_switches(&joystick.device, &joystick.switch_map, &mut joystick.state);

        println!(
            "Opened RC joystick '{}' at {:?}",
            joystick.device.name().unwrap_or("unknown device"),
            path
        );
        Ok(joystick)
    }

    /// Blocks until new events arrive and returns an updated frame when something changed.
    pub fn next_frame(&mut self) -> io::Result<Option<RcFrame>> {
        let mut updated = false;
        let events = self
            .device
            .fetch_events()
            .map_err(Error::other)?
            .collect::<Vec<_>>();

        for ev in events {
            match ev.destructure() {
                EventSummary::AbsoluteAxis(_, axis, raw) => {
                    let Some((role, scale)) = self.axis_map.get(&axis.0).copied() else {
                        continue;
                    };

                    let mut value = normalize_axis(&self.device, axis.0, raw);
                    if matches!(scale, AxisScale::ZeroToOne) {
                        value = ((value + 1.0) * 0.5).clamp(0.0, 1.0);
                    }
                    let role_changed = self.update_axis(role, value);
                    updated |= role_changed;
                }
                EventSummary::Key(_, key, raw) => {
                    if let Some(idx) = self.switch_map.get(&key.0) {
                        if let Some(sw) = self.state.switches.get_mut(*idx) {
                            let new_state = raw != 0;
                            if sw.on != new_state {
                                sw.on = new_state;
                                updated = true;
                            }
                        }
                    }
                }
                _ => {}
            }
        }

        if updated {
            Ok(Some(self.state.clone()))
        } else {
            Ok(None)
        }
    }

    /// Convenience helper that logs changes to stdout.
    pub fn print_loop(&mut self) -> io::Result<()> {
        println!("Waiting for RC joystick events...");
        loop {
            match self.next_frame()? {
                Some(frame) => {
                    let aux_values = frame
                        .aux
                        .iter()
                        .map(|a| format!("{}:{:+.2}", a.name, a.value))
                        .collect::<Vec<_>>()
                        .join(" ");
                    let switch_values = frame
                        .switches
                        .iter()
                        .map(|s| format!("{}:{}", s.name, if s.on { "on" } else { "off" }))
                        .collect::<Vec<_>>()
                        .join(" ");
                    println!(
                        "roll {:+.2} pitch {:+.2} yaw {:+.2} throttle {:+.2} sa {:+.2} sb {:+.2} sc {:+.2} | {} | {}",
                        frame.roll,
                        frame.pitch,
                        frame.yaw,
                        frame.throttle,
                        frame.knob_sa,
                        frame.knob_sb,
                        frame.knob_sc,
                        aux_values,
                        switch_values
                    );
                }
                None => thread::sleep(Duration::from_millis(10)),
            }
        }
    }

    fn update_axis(&mut self, role: RcAxis, value: f32) -> bool {
        match role {
            RcAxis::Roll => {
                let changed = !float_eq(self.state.roll, value);
                self.state.roll = value;
                changed
            }
            RcAxis::Pitch => {
                let changed = !float_eq(self.state.pitch, value);
                self.state.pitch = value;
                changed
            }
            RcAxis::Yaw => {
                let changed = !float_eq(self.state.yaw, value);
                self.state.yaw = value;
                changed
            }
            RcAxis::Throttle => {
                let changed = !float_eq(self.state.throttle, value);
                self.state.throttle = value;
                changed
            }
            RcAxis::KnobSa => {
                let changed = !float_eq(self.state.knob_sa, value);
                self.state.knob_sa = value;
                changed
            }
            RcAxis::KnobSb => {
                let changed = !float_eq(self.state.knob_sb, value);
                self.state.knob_sb = value;
                changed
            }
            RcAxis::KnobSc => {
                let changed = !float_eq(self.state.knob_sc, value);
                self.state.knob_sc = value;
                changed
            }
            RcAxis::Aux(i) => {
                if let Some(aux) = self.state.aux.get_mut(i as usize) {
                    let changed = !float_eq(aux.value, value);
                    aux.value = value;
                    changed
                } else {
                    false
                }
            }
        }
    }
}

fn float_eq(a: f32, b: f32) -> bool {
    (a - b).abs() < 0.0001
}

fn prime_axes(device: &Device, axis_map: &HashMap<u16, (RcAxis, AxisScale)>, state: &mut RcFrame) {
    if let Ok(abs_vals) = device.get_abs_state() {
        for (code, (role, scale)) in axis_map {
            let raw = abs_vals
                .get(*code as usize)
                .map(|info| info.value)
                .unwrap_or(0);
            let mut value = normalize_axis(device, *code, raw);
            if matches!(scale, AxisScale::ZeroToOne) {
                value = ((value + 1.0) * 0.5).clamp(0.0, 1.0);
            }
            match role {
                RcAxis::Roll => state.roll = value,
                RcAxis::Pitch => state.pitch = value,
                RcAxis::Yaw => state.yaw = value,
                RcAxis::Throttle => state.throttle = value,
                RcAxis::KnobSa => state.knob_sa = value,
                RcAxis::KnobSb => state.knob_sb = value,
                RcAxis::KnobSc => state.knob_sc = value,
                RcAxis::Aux(i) => {
                    if let Some(aux) = state.aux.get_mut(*i as usize) {
                        aux.value = value;
                    }
                }
            }
        }
    }
}

fn prime_switches(device: &Device, switch_map: &HashMap<u16, usize>, state: &mut RcFrame) {
    if let Ok(keys) = device.get_key_state() {
        for (code, idx) in switch_map {
            if let Some(sw) = state.switches.get_mut(*idx) {
                sw.on = keys.contains(KeyCode::new(*code));
            }
        }
    }
}

fn joystick_device_name(device: &Device) -> String {
    device.name().map(|n| n.to_lowercase()).unwrap_or_default()
}

// ExpressLRS joystick switch codes mapped to radio labels directly.
// Update these names if your Lua script exports different labels.
const ELRS_SWITCH_CODES: &[(KeyCode, &str)] =
    &[(KeyCode::new(0x120), "se"), (KeyCode::new(0x121), "sf")];

fn build_switches(device: &Device) -> Vec<SwitchState> {
    let name = joystick_device_name(device);
    let supported = device.supported_keys();

    if name.contains("expresslrs") || name.contains("radiomaster") {
        return ELRS_SWITCH_CODES
            .iter()
            .filter(|(code, _)| supported.is_some_and(|s| s.contains(*code)))
            .map(|(_, name)| SwitchState {
                name: name.to_string(),
                on: false,
            })
            .collect();
    }

    // Generic fallback: name buttons as btn1.. in supported order.
    let mut switches = Vec::new();
    if let Some(keys) = supported {
        for (idx, _key) in keys.iter().enumerate() {
            switches.push(SwitchState {
                name: format!("btn{}", idx + 1),
                on: false,
            });
        }
    }
    switches
}

fn build_switch_map(device: &Device, switches: &[SwitchState]) -> HashMap<u16, usize> {
    let mut map = HashMap::new();
    let device_name = joystick_device_name(device);
    if device_name.contains("expresslrs") || device_name.contains("radiomaster") {
        for (code, name) in ELRS_SWITCH_CODES.iter() {
            if let Some(idx) = switches.iter().position(|s| s.name == *name) {
                map.insert(code.0, idx);
            }
        }
    } else if let Some(keys) = device.supported_keys() {
        // Fallback: map supported keys in order to switches.
        for (idx, key) in keys.iter().enumerate() {
            if idx < switches.len() {
                map.insert(key.code(), idx);
            }
        }
    }
    map
}

fn discover_device(preferred_name: Option<&str>) -> io::Result<(PathBuf, Device)> {
    let mut best: Option<(i32, PathBuf, Device)> = None;
    for (path, device) in evdev::enumerate() {
        let score = score_device(&device, preferred_name);
        if score <= 0 {
            continue;
        }
        match &mut best {
            Some((best_score, _, _)) if score <= *best_score => {}
            _ => best = Some((score, path, device)),
        }
    }

    best.map(|(_, path, device)| (path, device))
        .ok_or_else(|| Error::new(ErrorKind::NotFound, "no RC joystick device found"))
}

fn score_device(device: &Device, preferred_name: Option<&str>) -> i32 {
    let mut score = 0;
    if device.supported_events().contains(EventType::ABSOLUTE) {
        score += 10;
    }

    if let Some(abs_axes) = device.supported_absolute_axes() {
        if abs_axes.iter().len() >= 4 {
            score += 10;
        }
    }

    if let Some(keys) = device.supported_keys() {
        let has_pad_like = keys.iter().any(|k| (0x120..=0x13e).contains(&k.code()));
        if has_pad_like {
            score += 5;
        }
    }

    if let Some(name) = device.name() {
        let lower = name.to_lowercase();
        if lower.contains("radio") || lower.contains("rc") || lower.contains("tx") {
            score += 20;
        }
        if lower.contains("joystick") {
            score += 5;
        }
        if let Some(pref) = preferred_name {
            if lower.contains(&pref.to_lowercase()) {
                score += 100;
            }
        }
    }

    score
}

fn build_axis_map(device: &Device) -> HashMap<u16, (RcAxis, AxisScale)> {
    let mut map = HashMap::new();
    let Some(axes) = device.supported_absolute_axes() else {
        return map;
    };

    if let Some(name) = device.name().map(|n| n.to_lowercase()) {
        if name.contains("expresslrs") || name.contains("radiomaster") {
            insert_if_present(
                axes,
                &mut map,
                AbsoluteAxisCode::ABS_X,
                RcAxis::Roll,
                AxisScale::NegOneToOne,
            );
            insert_if_present(
                axes,
                &mut map,
                AbsoluteAxisCode::ABS_Y,
                RcAxis::Pitch,
                AxisScale::NegOneToOne,
            );
            insert_if_present(
                axes,
                &mut map,
                AbsoluteAxisCode::ABS_RY,
                RcAxis::Yaw,
                AxisScale::NegOneToOne,
            );
            insert_if_present(
                axes,
                &mut map,
                AbsoluteAxisCode::ABS_RX,
                RcAxis::Throttle,
                AxisScale::ZeroToOne,
            );
            insert_if_present(
                axes,
                &mut map,
                AbsoluteAxisCode::ABS_Z,
                RcAxis::KnobSa,
                AxisScale::NegOneToOne,
            );
            insert_if_present(
                axes,
                &mut map,
                AbsoluteAxisCode::ABS_RZ,
                RcAxis::KnobSb,
                AxisScale::NegOneToOne,
            );
            insert_if_present(
                axes,
                &mut map,
                AbsoluteAxisCode::ABS_THROTTLE,
                RcAxis::KnobSc,
                AxisScale::NegOneToOne,
            );

            // Any remaining axes become aux channels.
            let mut aux_index = 0u8;
            for axis in axes.iter() {
                let code = axis.0;
                if let std::collections::hash_map::Entry::Vacant(entry) = map.entry(code) {
                    entry.insert((RcAxis::Aux(aux_index), AxisScale::NegOneToOne));
                    aux_index += 1;
                }
            }
            return map;
        }
    }

    bind_first(
        axes,
        &mut map,
        &[AbsoluteAxisCode::ABS_X, AbsoluteAxisCode::ABS_RX],
        RcAxis::Roll,
        AxisScale::NegOneToOne,
    );
    bind_first(
        axes,
        &mut map,
        &[AbsoluteAxisCode::ABS_Y, AbsoluteAxisCode::ABS_RY],
        RcAxis::Pitch,
        AxisScale::NegOneToOne,
    );
    bind_first(
        axes,
        &mut map,
        &[AbsoluteAxisCode::ABS_RZ, AbsoluteAxisCode::ABS_Z],
        RcAxis::Yaw,
        AxisScale::NegOneToOne,
    );
    bind_first(
        axes,
        &mut map,
        &[
            AbsoluteAxisCode::ABS_THROTTLE,
            AbsoluteAxisCode::ABS_Z,
            AbsoluteAxisCode::ABS_RUDDER,
        ],
        RcAxis::Throttle,
        AxisScale::ZeroToOne,
    );

    let mut aux_index = 0u8;
    for axis in axes.iter() {
        let code = axis.0;
        if let std::collections::hash_map::Entry::Vacant(entry) = map.entry(code) {
            entry.insert((RcAxis::Aux(aux_index), AxisScale::NegOneToOne));
            aux_index += 1;
        }
    }

    map
}

fn insert_if_present(
    axes: &AttributeSetRef<AbsoluteAxisCode>,
    map: &mut HashMap<u16, (RcAxis, AxisScale)>,
    axis: AbsoluteAxisCode,
    role: RcAxis,
    scale: AxisScale,
) {
    if axes.contains(axis) {
        map.insert(axis.0, (role, scale));
    }
}

fn bind_first(
    axes: &AttributeSetRef<AbsoluteAxisCode>,
    map: &mut HashMap<u16, (RcAxis, AxisScale)>,
    candidates: &[AbsoluteAxisCode],
    role: RcAxis,
    scale: AxisScale,
) {
    for axis in candidates {
        if axes.contains(*axis) && !map.contains_key(&axis.0) {
            map.insert(axis.0, (role, scale));
            break;
        }
    }
}

fn normalize_axis(device: &Device, axis_code: u16, raw: i32) -> f32 {
    if let Ok(abs_vals) = device.get_abs_state() {
        if let Some(info) = abs_vals.get(axis_code as usize) {
            let min = info.minimum as f32;
            let max = info.maximum as f32;
            if (max - min).abs() > f32::EPSILON {
                let normalized = (raw as f32 - min) / (max - min);
                return (normalized * 2.0 - 1.0).clamp(-1.0, 1.0);
            }
        }
    }

    let raw = raw as f32;
    (raw / 32767.0).clamp(-1.0, 1.0)
}
