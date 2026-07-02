#![cfg(any(feature = "sim", feature = "bevymon"))]

use std::collections::HashMap;
use std::io::{self, Error, ErrorKind};
use std::path::PathBuf;

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

/// AUX channel value.
#[derive(Debug, Clone)]
pub struct AuxChannel {
    pub value: f32,
}

/// Switch/button state with a stable name.
#[derive(Debug, Clone)]
pub struct SwitchState {
    pub name: String,
    pub on: bool,
}

#[derive(Debug, Clone, Default)]
pub struct RcAxisBindings {
    pub roll: Option<String>,
    pub pitch: Option<String>,
    pub yaw: Option<String>,
    pub throttle: Option<String>,
    pub knob_sa: Option<String>,
    pub knob_sb: Option<String>,
    pub knob_sc: Option<String>,
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
        let allow_generic = env_flag_true("CU_SIM_ALLOW_GENERIC_JOYSTICK");
        let (_path, device) = discover_device(preferred_name, allow_generic)?;

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
        for _ in 0..aux_count {
            aux.push(AuxChannel { value: 0.0 });
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

        joystick.device.set_nonblocking(true)?;
        prime_axes(&joystick.device, &joystick.axis_map, &mut joystick.state);
        prime_switches(&joystick.device, &joystick.switch_map, &mut joystick.state);

        Ok(joystick)
    }

    /// Blocks until new events arrive and returns an updated frame when something changed.
    pub fn next_frame(&mut self) -> io::Result<Option<RcFrame>> {
        let mut updated = false;
        let events = match self.device.fetch_events() {
            Ok(events) => events.collect::<Vec<_>>(),
            Err(err) if err.kind() == ErrorKind::WouldBlock => return Ok(None),
            Err(err) => return Err(err),
        };

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
                    if let Some(idx) = self.switch_map.get(&key.0)
                        && let Some(sw) = self.state.switches.get_mut(*idx)
                    {
                        let new_state = raw != 0;
                        if sw.on != new_state {
                            sw.on = new_state;
                            updated = true;
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

    /// Returns the latest known joystick state snapshot.
    pub fn current_frame(&self) -> RcFrame {
        self.state.clone()
    }

    /// Returns technical axis names currently bound to each RC role.
    pub fn axis_bindings(&self) -> RcAxisBindings {
        let mut bindings = RcAxisBindings::default();
        for (code, (role, _scale)) in &self.axis_map {
            let label = axis_code_label(*code);
            match role {
                RcAxis::Roll => bindings.roll = Some(label),
                RcAxis::Pitch => bindings.pitch = Some(label),
                RcAxis::Yaw => bindings.yaw = Some(label),
                RcAxis::Throttle => bindings.throttle = Some(label),
                RcAxis::KnobSa => bindings.knob_sa = Some(label),
                RcAxis::KnobSb => bindings.knob_sb = Some(label),
                RcAxis::KnobSc => bindings.knob_sc = Some(label),
                RcAxis::Aux(_) => {}
            }
        }
        bindings
    }

    /// Returns the input device name used for RC mapping.
    pub fn device_name(&self) -> String {
        self.device.name().unwrap_or("unknown").to_string()
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

fn is_radio_profile_name(name: &str) -> bool {
    is_elrs_profile_name(name) || is_opentx_profile_name(name)
}

fn is_elrs_profile_name(name: &str) -> bool {
    name.contains("expresslrs") || name.contains("radiomaster")
}

fn is_opentx_profile_name(name: &str) -> bool {
    name.contains("opentx") || name.contains("edgetx")
}

// ExpressLRS joystick switch codes mapped to radio labels directly.
// Update these names if your Lua script exports different labels.
const ELRS_SWITCH_CODES: &[(KeyCode, &str)] =
    &[(KeyCode::new(0x120), "se"), (KeyCode::new(0x121), "sf")];

fn build_switches(device: &Device) -> Vec<SwitchState> {
    let name = joystick_device_name(device);
    let supported = device.supported_keys();

    if is_radio_profile_name(&name) {
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
    if is_radio_profile_name(&device_name) {
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

fn discover_device(
    preferred_name: Option<&str>,
    allow_generic: bool,
) -> io::Result<(PathBuf, Device)> {
    let mut best: Option<(i32, PathBuf, Device)> = None;
    for (path, device) in evdev::enumerate() {
        let score = score_device(&device, preferred_name, allow_generic);
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

fn score_device(device: &Device, preferred_name: Option<&str>, allow_generic: bool) -> i32 {
    let mut score = 0;
    if device.supported_events().contains(EventType::ABSOLUTE) {
        score += 10;
    }

    if let Some(abs_axes) = device.supported_absolute_axes()
        && abs_axes.iter().len() >= 4
    {
        score += 10;
    }

    if let Some(keys) = device.supported_keys()
        && keys.iter().any(|k| (0x120..=0x13e).contains(&k.code()))
    {
        score += 5;
    }

    let lower = joystick_device_name(device);
    if let Some(pref) = preferred_name
        && lower.contains(&pref.to_lowercase())
    {
        return score + 1000;
    }

    let is_radio = is_radio_profile_name(&lower);
    if is_radio {
        score += 40;
    } else if !allow_generic {
        return 0;
    } else if lower.contains("joystick") {
        score += 5;
    }

    if lower.contains("radio") || lower.contains("rc") || lower.contains("tx") {
        score += 20;
    }

    score
}

fn build_axis_map(device: &Device) -> HashMap<u16, (RcAxis, AxisScale)> {
    let mut map = HashMap::new();
    let Some(axes) = device.supported_absolute_axes() else {
        return map;
    };

    if let Some(name) = device.name().map(|n| n.to_lowercase())
        && is_elrs_profile_name(&name)
    {
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

    if let Some(name) = device.name().map(|n| n.to_lowercase())
        && is_opentx_profile_name(&name)
    {
        // OpenTX/EdgeTX USB joystick layout observed on TX15.
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
            AbsoluteAxisCode::ABS_RX,
            RcAxis::Yaw,
            AxisScale::NegOneToOne,
        );
        insert_if_present(
            axes,
            &mut map,
            AbsoluteAxisCode::ABS_Z,
            RcAxis::Throttle,
            AxisScale::ZeroToOne,
        );
        insert_if_present(
            axes,
            &mut map,
            AbsoluteAxisCode::ABS_RY,
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
    if let Ok(abs_vals) = device.get_abs_state()
        && let Some(info) = abs_vals.get(axis_code as usize)
    {
        let min = info.minimum as f32;
        let max = info.maximum as f32;
        let range = max - min;
        if range.abs() > f32::EPSILON {
            let normalized = (raw as f32 - min) / range;
            return (normalized * 2.0 - 1.0).clamp(-1.0, 1.0);
        }
    }

    let raw = raw as f32;
    (raw / 32767.0).clamp(-1.0, 1.0)
}

fn axis_code_label(code: u16) -> String {
    match code {
        x if x == AbsoluteAxisCode::ABS_X.0 => "ABS_X".to_string(),
        x if x == AbsoluteAxisCode::ABS_Y.0 => "ABS_Y".to_string(),
        x if x == AbsoluteAxisCode::ABS_Z.0 => "ABS_Z".to_string(),
        x if x == AbsoluteAxisCode::ABS_RX.0 => "ABS_RX".to_string(),
        x if x == AbsoluteAxisCode::ABS_RY.0 => "ABS_RY".to_string(),
        x if x == AbsoluteAxisCode::ABS_RZ.0 => "ABS_RZ".to_string(),
        x if x == AbsoluteAxisCode::ABS_THROTTLE.0 => "ABS_THROTTLE".to_string(),
        x if x == AbsoluteAxisCode::ABS_RUDDER.0 => "ABS_RUDDER".to_string(),
        x if x == AbsoluteAxisCode::ABS_WHEEL.0 => "ABS_WHEEL".to_string(),
        _ => format!("ABS_0x{code:02X}"),
    }
}

fn env_flag_true(name: &str) -> bool {
    std::env::var(name).is_ok_and(|v| {
        let v = v.trim();
        v == "1"
            || v.eq_ignore_ascii_case("true")
            || v.eq_ignore_ascii_case("yes")
            || v.eq_ignore_ascii_case("on")
    })
}
