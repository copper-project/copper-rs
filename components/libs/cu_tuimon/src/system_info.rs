#[cfg(all(target_family = "wasm", target_os = "unknown"))]
use ratatui::text::Text;

#[derive(Clone)]
pub(crate) enum SystemInfo {
    #[cfg(not(all(target_family = "wasm", target_os = "unknown")))]
    Ansi(String),
    #[cfg(all(target_family = "wasm", target_os = "unknown"))]
    Rich(Text<'static>),
}

#[cfg(not(all(target_family = "wasm", target_os = "unknown")))]
mod native {
    use crate::system_info::SystemInfo;
    use libmacchina::{
        GeneralReadout, KernelReadout, MemoryReadout, traits::GeneralReadout as _,
        traits::KernelReadout as _, traits::MemoryReadout as _,
    };
    use pfetch_logo_parser::{Color, Logo, LogoPart};
    use std::{env, fmt::Display, str::FromStr};

    #[derive(Debug, PartialEq)]
    enum PfetchInfo {
        Os,
        Host,
        Kernel,
        Uptime,
        Cpu,
        Memory,
        Shell,
        Editor,
        Wm,
        De,
    }

    impl Display for PfetchInfo {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(f, "{}", format!("{self:?}").to_lowercase())
        }
    }

    impl FromStr for PfetchInfo {
        type Err = String;

        fn from_str(info: &str) -> Result<Self, Self::Err> {
            match info {
                "os" => Ok(PfetchInfo::Os),
                "host" => Ok(PfetchInfo::Host),
                "kernel" => Ok(PfetchInfo::Kernel),
                "uptime" => Ok(PfetchInfo::Uptime),
                "cpu" => Ok(PfetchInfo::Cpu),
                "memory" => Ok(PfetchInfo::Memory),
                "shell" => Ok(PfetchInfo::Shell),
                "editor" => Ok(PfetchInfo::Editor),
                "wm" => Ok(PfetchInfo::Wm),
                "de" => Ok(PfetchInfo::De),
                unknown_info => Err(format!("Unknown pfetch info: {unknown_info}")),
            }
        }
    }

    struct Readouts {
        general_readout: GeneralReadout,
        memory_readout: MemoryReadout,
        kernel_readout: KernelReadout,
    }

    impl Default for Readouts {
        fn default() -> Self {
            Readouts {
                general_readout: GeneralReadout::new(),
                memory_readout: MemoryReadout::new(),
                kernel_readout: KernelReadout::new(),
            }
        }
    }

    fn get_info(info: &PfetchInfo, readouts: &Readouts) -> Option<String> {
        match info {
            PfetchInfo::Os => pfetch::os(&readouts.general_readout),
            PfetchInfo::Host => pfetch::host(&readouts.general_readout),
            PfetchInfo::Kernel => pfetch::kernel(&readouts.kernel_readout),
            PfetchInfo::Uptime => pfetch::uptime(&readouts.general_readout),
            PfetchInfo::Cpu => pfetch::cpu(&readouts.general_readout),
            PfetchInfo::Memory => pfetch::memory(&readouts.memory_readout),
            PfetchInfo::Shell => pfetch::shell(&readouts.general_readout),
            PfetchInfo::Editor => Some(env::var("EDITOR").unwrap_or_else(|_| "nvim".into())),
            PfetchInfo::Wm => pfetch::wm(&readouts.general_readout),
            PfetchInfo::De => pfetch::de(&readouts.general_readout),
        }
    }

    pub(crate) fn default_system_info() -> SystemInfo {
        let readouts = Readouts::default();

        let os = pfetch::os(&readouts.general_readout).unwrap_or_default();

        let all_infos = [
            PfetchInfo::Os,
            PfetchInfo::Host,
            PfetchInfo::Kernel,
            PfetchInfo::Uptime,
            PfetchInfo::Cpu,
            PfetchInfo::Memory,
            PfetchInfo::Shell,
            PfetchInfo::Editor,
            PfetchInfo::Wm,
            PfetchInfo::De,
        ];

        let logo = pfetch::logo(&os);
        let gathered_pfetch_info: Vec<(Color, String, String)> = all_infos
            .iter()
            .filter_map(|info| match info {
                PfetchInfo::Os => Some((logo.primary_color, info.to_string(), os.clone())),
                _ => get_info(info, &readouts)
                    .map(|info_str| (logo.primary_color, info.to_string(), info_str)),
            })
            .collect();

        SystemInfo::Ansi(pfetch(gathered_pfetch_info, logo, true))
    }

    fn env_usize(name: &str, default: usize) -> usize {
        dotenvy::var(name)
            .ok()
            .and_then(|value| value.parse::<usize>().ok())
            .unwrap_or(default)
    }

    fn pfetch(info: Vec<(Color, String, String)>, logo: Logo, logo_enabled: bool) -> String {
        let raw_logo = if logo_enabled {
            logo.logo_parts
                .iter()
                .map(|LogoPart { content, .. }| content.as_ref())
                .collect::<String>()
        } else {
            "".into()
        };
        let color_enabled = dotenvy::var("PF_COLOR").unwrap_or_default() != "0";
        let logo = if color_enabled {
            logo.to_string()
        } else {
            format!("{logo:#}")
        };
        let mut logo_lines = logo.lines();
        let raw_logo_lines: Vec<_> = raw_logo.lines().collect();
        let logo_width = raw_logo_lines
            .iter()
            .map(|line| line.chars().count())
            .max()
            .unwrap_or(0);
        let line_amount = usize::max(raw_logo_lines.len(), info.len());

        let info1_width = info
            .iter()
            .skip(1)
            .map(|(_, line, _)| {
                if line.starts_with("\x1b[4") {
                    0
                } else {
                    line.len()
                }
            })
            .max()
            .unwrap_or(0);

        let padding1 = env_usize("PF_PAD1", 0);
        let padding2 = env_usize("PF_PAD2", 3);
        let padding3 = env_usize("PF_PAD3", 1);

        let mut pfetch_str = String::new();

        for l in 0..line_amount {
            pfetch_str += &format!(
                "{padding1}{bold}{logo}{padding2}{color}{info1}{nobold}{separator}{padding3}{color2}{info2}\n",
                padding1 = " ".repeat(padding1),
                bold = if color_enabled { "\x1b[1m" } else { "" },
                logo = if logo_enabled {
                    logo_lines.next().unwrap_or("")
                } else {
                    ""
                },
                padding2 = " ".repeat(
                    logo_width - raw_logo_lines.get(l).map_or(0, |line| line.chars().count())
                        + if logo_enabled { padding2 } else { 0 }
                ),
                color = if color_enabled {
                    info.get(l).map_or("".to_owned(), |line| {
                        let (color, _, _) = line;
                        color.to_string()
                    })
                } else {
                    "".into()
                },
                info1 = info.get(l).map_or("", |line| &line.1),
                nobold = if color_enabled { "\x1b[0m" } else { "" },
                separator = info.get(l).map_or("".into(), |line| {
                    if !&line.2.is_empty() {
                        dotenvy::var("PF_SEP").unwrap_or_default()
                    } else {
                        "".into()
                    }
                }),
                padding3 = " ".repeat(
                    info1_width.saturating_sub(info.get(l).map_or(0, |(_, line, _)| line.len()))
                        + padding3
                ),
                color2 = if color_enabled {
                    match dotenvy::var("PF_COL2") {
                        Ok(newcolor) => match Color::from_str(&newcolor) {
                            Ok(newcolor) => format!("{newcolor}"),
                            Err(_) => "".into(),
                        },
                        Err(_) => "".into(),
                    }
                } else {
                    "".into()
                },
                info2 = info.get(l).map_or("", |line| &line.2)
            )
        }

        pfetch_str
    }
}

#[cfg(all(target_family = "wasm", target_os = "unknown"))]
mod browser {
    use crate::system_info::SystemInfo;
    use js_sys::{Array, Intl, Reflect};
    use ratatui::{
        style::{Color, Modifier, Style},
        text::{Line, Span, Text},
    };
    use wasm_bindgen::JsValue;
    use web_sys::{Navigator, Screen, window};

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum BrowserBrand {
        Firefox,
        Chrome,
        Chromium,
        Safari,
        Edge,
        Tor,
        Opera,
        Vivaldi,
        Brave,
        Unknown,
    }

    impl BrowserBrand {
        fn from_env(navigator: &Navigator, user_agent: &str) -> Self {
            if has_brave(navigator) {
                Self::Brave
            } else if user_agent.contains("TorBrowser/") {
                Self::Tor
            } else if user_agent.contains("FxiOS/") || user_agent.contains("Firefox/") {
                Self::Firefox
            } else if user_agent.contains("Edg/")
                || user_agent.contains("EdgA/")
                || user_agent.contains("EdgiOS/")
            {
                Self::Edge
            } else if user_agent.contains("OPR/")
                || user_agent.contains("OPiOS/")
                || user_agent.contains("Opera/")
            {
                Self::Opera
            } else if user_agent.contains("Vivaldi/") {
                Self::Vivaldi
            } else if user_agent.contains("Chromium/") {
                Self::Chromium
            } else if user_agent.contains("CriOS/") || user_agent.contains("Chrome/") {
                Self::Chrome
            } else if user_agent.contains("Safari/") && user_agent.contains("Version/") {
                Self::Safari
            } else {
                Self::Unknown
            }
        }

        fn display_name(self) -> &'static str {
            match self {
                Self::Firefox => "Firefox",
                Self::Chrome => "Chrome",
                Self::Chromium => "Chromium",
                Self::Safari => "Safari",
                Self::Edge => "Edge",
                Self::Tor => "Tor Browser",
                Self::Opera => "Opera",
                Self::Vivaldi => "Vivaldi",
                Self::Brave => "Brave",
                Self::Unknown => "Browser",
            }
        }

        fn engine(self) -> &'static str {
            match self {
                Self::Firefox | Self::Tor => "Gecko",
                Self::Safari => "WebKit",
                Self::Chrome
                | Self::Chromium
                | Self::Edge
                | Self::Opera
                | Self::Vivaldi
                | Self::Brave => "Blink",
                Self::Unknown => "Web",
            }
        }

        fn logo(self) -> Vec<Line<'static>> {
            match self {
                Self::Firefox => vec![
                    colored_logo_line(Color::Indexed(208), "       /\\     "),
                    colored_logo_line(Color::Indexed(202), "    .-'  '-.  "),
                    colored_logo_line(Color::Indexed(208), "   /  .--.  \\"),
                    colored_logo_line(Color::Indexed(214), "  ;  / () \\  ;"),
                    colored_logo_line(Color::Indexed(129), "  |  \\    /  |"),
                    colored_logo_line(Color::Indexed(93), "  ;   '--'   ;"),
                    colored_logo_line(Color::Indexed(57), "   \\  .__.  / "),
                    colored_logo_line(Color::Indexed(57), "    '-.__.-'  "),
                ],
                Self::Chrome => vec![
                    colored_logo_line(Color::Indexed(196), "      .-==-."),
                    colored_logo_line(Color::Indexed(196), "   .-'  ()  '-."),
                    colored_logo_line(Color::Indexed(226), "  /  .-====-.  \\"),
                    colored_logo_line(Color::Indexed(226), " ;  /  ()    \\  ;"),
                    colored_logo_line(Color::Indexed(46), " | |   ----   | |"),
                    colored_logo_line(Color::Indexed(46), " ;  \\        /  ;"),
                    colored_logo_line(Color::Indexed(46), "  \\  '-.__.-'  /"),
                    colored_logo_line(Color::Indexed(196), "   '-.______.-"),
                ],
                Self::Chromium => vec![
                    colored_logo_line(Color::Indexed(33), "      .-==-."),
                    colored_logo_line(Color::Indexed(39), "   .-'  ..  '-."),
                    colored_logo_line(Color::Indexed(45), "  /  .-====-.  \\"),
                    colored_logo_line(Color::Indexed(39), " ;  /   __   \\  ;"),
                    colored_logo_line(Color::Indexed(33), " | |   /  \\   | |"),
                    colored_logo_line(Color::Indexed(39), " ;  \\  \\__/  /  ;"),
                    colored_logo_line(Color::Indexed(45), "  \\  '-.__.-'  /"),
                    colored_logo_line(Color::Indexed(39), "   '-.______.-"),
                ],
                Self::Safari => vec![
                    colored_logo_line(Color::Indexed(39), "     .------."),
                    colored_logo_line(Color::Indexed(45), "   .'  /\\    '."),
                    colored_logo_line(Color::Indexed(39), "  /   /  \\     \\"),
                    colored_logo_line(Color::Indexed(33), " ;    \\  /     ;"),
                    colored_logo_line(Color::Indexed(27), " |     \\/      |"),
                    colored_logo_line(Color::Indexed(33), " ;     /\\      ;"),
                    colored_logo_line(Color::Indexed(39), "  \\   /  \\    /"),
                    colored_logo_line(Color::Indexed(45), "   '._______.'"),
                ],
                Self::Edge => vec![
                    colored_logo_line(Color::Indexed(51), "      ______"),
                    colored_logo_line(Color::Indexed(45), "   .-'  __  '-."),
                    colored_logo_line(Color::Indexed(39), "  /   .'  '.   \\"),
                    colored_logo_line(Color::Indexed(43), " ;   /  __  \\   ;"),
                    colored_logo_line(Color::Indexed(33), " |  |  /  \\  |  |"),
                    colored_logo_line(Color::Indexed(42), " ;   \\ \\__/ /   ;"),
                    colored_logo_line(Color::Indexed(36), "  \\   '.__.'   /"),
                    colored_logo_line(Color::Indexed(45), "   '-.______.-'"),
                ],
                Self::Tor => vec![
                    colored_logo_line(Color::Indexed(135), "       .--."),
                    colored_logo_line(Color::Indexed(141), "     .'_\\/_.'"),
                    colored_logo_line(Color::Indexed(135), "    / / /\\ \\ \\"),
                    colored_logo_line(Color::Indexed(99), "   ; | |  | | ;"),
                    colored_logo_line(Color::Indexed(93), "   | | |  | | |"),
                    colored_logo_line(Color::Indexed(99), "   ; | |__| | ;"),
                    colored_logo_line(Color::Indexed(135), "    \\ \\____/ /"),
                    colored_logo_line(Color::Indexed(141), "     '.___.'"),
                ],
                Self::Opera => vec![
                    colored_logo_line(Color::Indexed(197), "          .-======-.  "),
                    colored_logo_line(Color::Indexed(197), "       .-'  .--.   '-."),
                    colored_logo_line(Color::Indexed(197), "      /   .'    '.   \\"),
                    colored_logo_line(Color::Indexed(197), "     |   |  .--.  |   |"),
                    colored_logo_line(Color::Indexed(197), "     |   |  '--'  |   |"),
                    colored_logo_line(Color::Indexed(197), "      \\   '.____.'   /"),
                    colored_logo_line(Color::Indexed(197), "       '-. Opera .-' "),
                ],
                Self::Vivaldi => vec![
                    colored_logo_line(Color::Indexed(204), "         .-========-. "),
                    colored_logo_line(Color::Indexed(204), "      .-'  _    _   '-."),
                    colored_logo_line(Color::Indexed(204), "     /   .' )  ( '.   \\"),
                    colored_logo_line(Color::Indexed(204), "    |   /  / /\\ \\  \\   |"),
                    colored_logo_line(Color::Indexed(204), "    |   \\  \\/  \\/  /   |"),
                    colored_logo_line(Color::Indexed(204), "     \\   '._/__\\_.'   /"),
                    colored_logo_line(Color::Indexed(204), "      '-. Vivaldi.-'  "),
                ],
                Self::Brave => vec![
                    colored_logo_line(Color::Indexed(208), "      /\\  /\\"),
                    colored_logo_line(Color::Indexed(202), "   .-(  \\/  )-."),
                    colored_logo_line(Color::Indexed(208), "  /   \\    /   \\"),
                    colored_logo_line(Color::Indexed(214), " ;     |  |     ;"),
                    colored_logo_line(Color::Indexed(220), " |     |  |     |"),
                    colored_logo_line(Color::Indexed(214), " ;     |  |     ;"),
                    colored_logo_line(Color::Indexed(208), "  \\    '--'    /"),
                    colored_logo_line(Color::Indexed(202), "   '-.______.-'"),
                ],
                Self::Unknown => vec![
                    colored_logo_line(Color::White, "      .----------------."),
                    colored_logo_line(Color::White, "     / .--------------. \\"),
                    colored_logo_line(Color::White, "    | |    Browser     | |"),
                    colored_logo_line(Color::White, "    | |  Copper Web    | |"),
                    colored_logo_line(Color::White, "    | |     Monitor    | |"),
                    colored_logo_line(Color::White, "     \\ '--------------' /"),
                    colored_logo_line(Color::White, "      '----------------' "),
                ],
            }
        }
    }

    pub(crate) fn default_system_info() -> SystemInfo {
        let Some(window) = window() else {
            return browser_fallback();
        };

        let navigator = window.navigator();
        let user_agent = result_string(navigator.user_agent()).unwrap_or_default();
        let brand = BrowserBrand::from_env(&navigator, &user_agent);
        let browser_version =
            extract_version(&user_agent, brand).unwrap_or_else(|| "unknown".to_string());
        let language = navigator
            .language()
            .unwrap_or_else(|| "unknown".to_string());
        let languages = join_languages(navigator.languages());
        let platform = result_string(navigator.platform()).unwrap_or_else(|| "unknown".to_string());
        let vendor =
            reflected_string(navigator.as_ref(), "vendor").unwrap_or_else(|| "unknown".to_string());
        let viewport = viewport_string(&window);
        let screen = window.screen().ok().map(screen_string);
        let timezone = timezone_string();
        let device_memory = device_memory_string(&navigator);
        let touch_points = max_touch_points(&navigator);

        let mut info = vec![
            (
                "browser".to_string(),
                format!("{} {}", brand.display_name(), browser_version),
            ),
            ("engine".to_string(), brand.engine().to_string()),
            ("platform".to_string(), platform),
            ("vendor".to_string(), vendor),
            ("language".to_string(), language),
            ("runtime".to_string(), "wasm32-unknown-unknown".to_string()),
        ];

        if !languages.is_empty() {
            info.push(("langs".to_string(), languages));
        }

        info.push((
            "online".to_string(),
            if navigator.on_line() {
                "yes".to_string()
            } else {
                "no".to_string()
            },
        ));

        let cores = navigator.hardware_concurrency();
        if cores > 0.0 {
            let cores_display = if cores.fract() == 0.0 {
                format!("{cores:.0}")
            } else {
                format!("{cores:.1}")
            };
            info.push(("cores".to_string(), cores_display));
        }

        if let Some(device_memory) = device_memory {
            info.push(("memory".to_string(), device_memory));
        }

        if touch_points > 0 {
            info.push(("touch".to_string(), touch_points.to_string()));
        }

        if let Some(viewport) = viewport {
            info.push(("viewport".to_string(), viewport));
        }

        if let Some(screen) = screen {
            info.push(("screen".to_string(), screen));
        }

        info.push((
            "dpr".to_string(),
            format!("{:.2}", window.device_pixel_ratio()),
        ));

        if let Some(timezone) = timezone {
            info.push(("tz".to_string(), timezone));
        }

        info.push(("agent".to_string(), user_agent));

        SystemInfo::Rich(format_browser_pfetch(brand, &info))
    }

    fn browser_fallback() -> SystemInfo {
        let info = vec![
            ("browser".to_string(), "Browser".to_string()),
            ("runtime".to_string(), "wasm32-unknown-unknown".to_string()),
            (
                "status".to_string(),
                "window() unavailable in this environment".to_string(),
            ),
        ];
        SystemInfo::Rich(format_browser_pfetch(BrowserBrand::Unknown, &info))
    }

    fn result_string(value: Result<String, JsValue>) -> Option<String> {
        value.ok().filter(|value| !value.trim().is_empty())
    }

    fn reflected_string(target: &JsValue, key: &str) -> Option<String> {
        Reflect::get(target, &JsValue::from_str(key))
            .ok()
            .and_then(|value| value.as_string())
            .filter(|value| !value.trim().is_empty())
    }

    fn join_languages(values: Array) -> String {
        values
            .iter()
            .filter_map(|value| value.as_string())
            .collect::<Vec<_>>()
            .join(", ")
    }

    fn viewport_string(window: &web_sys::Window) -> Option<String> {
        let width = window.inner_width().ok()?.as_f64()?;
        let height = window.inner_height().ok()?.as_f64()?;
        Some(format!("{width:.0}x{height:.0}"))
    }

    fn screen_string(screen: Screen) -> String {
        let width = screen.width().ok();
        let height = screen.height().ok();
        let avail_width = screen.avail_width().ok();
        let avail_height = screen.avail_height().ok();
        let color_depth = screen.color_depth().ok();

        let mut parts = Vec::new();
        if let (Some(width), Some(height)) = (width, height) {
            parts.push(format!("{width}x{height}"));
        }
        if let (Some(width), Some(height)) = (avail_width, avail_height) {
            parts.push(format!("avail {width}x{height}"));
        }
        if let Some(depth) = color_depth {
            parts.push(format!("{depth}-bit"));
        }
        parts.join(" | ")
    }

    fn timezone_string() -> Option<String> {
        let options = Intl::DateTimeFormat::default().resolved_options();
        Reflect::get(&options, &JsValue::from_str("timeZone"))
            .ok()
            .and_then(|value| value.as_string())
    }

    fn has_brave(navigator: &Navigator) -> bool {
        Reflect::get(navigator.as_ref(), &JsValue::from_str("brave"))
            .ok()
            .is_some_and(|value| !value.is_null() && !value.is_undefined())
    }

    fn device_memory_string(navigator: &Navigator) -> Option<String> {
        let memory = Reflect::get(navigator.as_ref(), &JsValue::from_str("deviceMemory"))
            .ok()?
            .as_f64()?;
        Some(if memory.fract() == 0.0 {
            format!("{memory:.0} GiB")
        } else {
            format!("{memory:.1} GiB")
        })
    }

    fn max_touch_points(navigator: &Navigator) -> u32 {
        Reflect::get(navigator.as_ref(), &JsValue::from_str("maxTouchPoints"))
            .ok()
            .and_then(|value| value.as_f64())
            .map_or(0, |value| value as u32)
    }

    fn extract_version(user_agent: &str, brand: BrowserBrand) -> Option<String> {
        let tokens = match brand {
            BrowserBrand::Firefox => &["Firefox/", "FxiOS/"][..],
            BrowserBrand::Chrome => &["Chrome/", "CriOS/"][..],
            BrowserBrand::Chromium => &["Chromium/"][..],
            BrowserBrand::Safari => &["Version/"][..],
            BrowserBrand::Edge => &["Edg/", "EdgA/", "EdgiOS/"][..],
            BrowserBrand::Tor => &["TorBrowser/", "Firefox/"][..],
            BrowserBrand::Opera => &["OPR/", "OPiOS/", "Opera/"][..],
            BrowserBrand::Vivaldi => &["Vivaldi/"][..],
            BrowserBrand::Brave => &["Brave/", "Chrome/", "CriOS/"][..],
            BrowserBrand::Unknown => &[][..],
        };

        for token in tokens {
            if let Some(version) = extract_version_token(user_agent, token) {
                return Some(version);
            }
        }
        None
    }

    fn extract_version_token(user_agent: &str, token: &str) -> Option<String> {
        let start = user_agent.find(token)? + token.len();
        let tail = &user_agent[start..];
        let end = tail
            .find(|ch: char| !(ch.is_ascii_digit() || ch == '.'))
            .unwrap_or(tail.len());
        let version = &tail[..end];
        if version.is_empty() {
            None
        } else {
            Some(version.to_string())
        }
    }

    fn format_browser_pfetch(brand: BrowserBrand, info: &[(String, String)]) -> Text<'static> {
        let logo = brand.logo();
        let logo_width = logo.iter().map(line_width).max().unwrap_or(0);
        let key_width = info.iter().map(|(key, _)| key.len()).max().unwrap_or(0);
        let line_count = logo.len().max(info.len());

        let mut output = Vec::with_capacity(line_count);
        for idx in 0..line_count {
            let mut line = logo.get(idx).cloned().unwrap_or_default();
            let padding = logo_width.saturating_sub(line_width(&line));

            if let Some((key, value)) = info.get(idx) {
                line.spans.push(Span::raw(" ".repeat(padding + 3)));
                line.spans.push(Span::styled(key.clone(), label_style()));
                line.spans.push(Span::raw(
                    " ".repeat(key_width.saturating_sub(key.len()) + 1),
                ));
                line.spans.push(Span::styled(value.clone(), value_style()));
            } else if padding > 0 {
                line.spans.push(Span::raw(" ".repeat(padding)));
            }

            output.push(line);
        }

        Text::from(output)
    }

    fn colored_logo_line(color: Color, text: &'static str) -> Line<'static> {
        let style = Style::default().fg(color);
        let mut spans = Vec::new();
        let mut buf = String::new();
        let mut buf_is_space: Option<bool> = None;

        for ch in text.chars() {
            let is_space = ch == ' ';
            match buf_is_space {
                Some(current) if current == is_space => buf.push(ch),
                Some(current) => {
                    spans.push(if current {
                        Span::raw(std::mem::take(&mut buf))
                    } else {
                        Span::styled(std::mem::take(&mut buf), style)
                    });
                    buf.push(ch);
                    buf_is_space = Some(is_space);
                }
                None => {
                    buf.push(ch);
                    buf_is_space = Some(is_space);
                }
            }
        }

        if let Some(current) = buf_is_space {
            spans.push(if current {
                Span::raw(buf)
            } else {
                Span::styled(buf, style)
            });
        }

        Line::from(spans)
    }

    fn line_width(line: &Line<'_>) -> usize {
        line.spans
            .iter()
            .map(|span| span.content.chars().count())
            .sum()
    }

    fn label_style() -> Style {
        Style::default()
            .fg(Color::Blue)
            .add_modifier(Modifier::BOLD)
    }

    fn value_style() -> Style {
        Style::default().fg(Color::White)
    }
}

#[cfg(not(all(target_family = "wasm", target_os = "unknown")))]
pub(crate) use native::default_system_info;

#[cfg(all(target_family = "wasm", target_os = "unknown"))]
pub(crate) use browser::default_system_info;
