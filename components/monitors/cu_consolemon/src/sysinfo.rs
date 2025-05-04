use libmacchina::{
    traits::GeneralReadout as _, traits::KernelReadout as _, traits::MemoryReadout as _,
    traits::PackageReadout as _, GeneralReadout, KernelReadout, MemoryReadout, PackageReadout,
};

use pfetch_logo_parser::{Color, Logo, LogoPart};
use std::{env, fmt::Display, str::FromStr};

#[derive(Debug, PartialEq)]
pub enum PfetchInfo {
    Ascii,
    Title,
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
    Palette,
    BlankLine,
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
            "ascii" => Ok(PfetchInfo::Ascii),
            "title" => Ok(PfetchInfo::Title),
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
            "palette" => Ok(PfetchInfo::Palette),
            unknown_info => Err(format!("Unknown pfetch info: {unknown_info}")),
        }
    }
}

// Struct for the various readouts
pub struct Readouts {
    pub general_readout: GeneralReadout,
    pub package_readout: PackageReadout,
    pub memory_readout: MemoryReadout,
    pub kernel_readout: KernelReadout,
}

impl Default for Readouts {
    fn default() -> Self {
        Readouts {
            general_readout: GeneralReadout::new(),
            package_readout: PackageReadout::new(),
            memory_readout: MemoryReadout::new(),
            kernel_readout: KernelReadout::new(),
        }
    }
}

pub fn get_info(info: &PfetchInfo, readouts: &Readouts) -> Option<String> {
    match info {
        PfetchInfo::Ascii => None,
        PfetchInfo::Title => pfetch::user_at_hostname(
            &readouts.general_readout,
            &dotenvy::var("USER").ok(),
            &dotenvy::var("HOSTNAME").ok(),
        ),
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
        PfetchInfo::Palette => Some("Color Palette".into()), // Simplified palette display
        PfetchInfo::BlankLine => Some("".into()),
    }
}

// Function to render the gathered info along with ASCII logo
pub fn pfetch_info() -> String {
    let readouts = Readouts::default();

    let os = pfetch::os(&GeneralReadout::new()).unwrap_or_default();

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
            _ => get_info(info, &readouts).map(|info_str| match info {
                PfetchInfo::Title => (logo.secondary_color, info_str, "".into()),
                PfetchInfo::BlankLine => (logo.primary_color, "".into(), "".into()),
                _ => (logo.primary_color, info.to_string(), info_str),
            }),
        })
        .collect();

    pfetch(gathered_pfetch_info, logo, true)
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
                // exclude palette from info1 width
                0
            } else {
                line.len()
            }
        })
        .max()
        .unwrap_or(0);

    let padding1 = match dotenvy::var("PF_PAD1") {
        Ok(padding0) => padding0.parse::<usize>().unwrap_or(0),
        Err(_) => 0,
    };
    let padding2 = match dotenvy::var("PF_PAD2") {
        Ok(padding1) => padding1.parse::<usize>().unwrap_or(0),
        Err(_) => 3,
    };
    let padding3 = match dotenvy::var("PF_PAD3") {
        Ok(padding2) => padding2.parse::<usize>().unwrap_or(0),
        Err(_) => 1,
    };

    let mut pfetch_str = String::new();

    for l in 0..line_amount {
        pfetch_str += &format!(
            "{padding1}{bold}{logo}{padding2}{color}{info1}{nobold}{separator}{padding3}{color2}{info2}\n",
            padding1 = " ".repeat(padding1),
            bold = if color_enabled {"\x1b[1m"} else {""},
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
                }
                )
            } else {
                "".into()
            },
            info1 = info.get(l).map_or("", |line| &line.1),
            nobold = if color_enabled {"\x1b[0m"} else {""},
            separator = info.get(l).map_or("".into(), |line|
                if ! &line.2.is_empty() {
                    dotenvy::var("PF_SEP").unwrap_or_default()
                } else { "".into() }
            ),
            padding3 = " ".repeat(
                info1_width.saturating_sub(info.get(l).map_or(0, |(_, line, _)| line.len()))
                    + padding3
            ),
            color2 = if color_enabled {match dotenvy::var("PF_COL2") {
                Ok(newcolor) => {
                    match Color::from_str(&newcolor) {
                        Ok(newcolor) => format!("{newcolor}"),
                        Err(_) => "".into(),
                    }
                },
                Err(_) => "".into()
            }} else {"".into()},
            info2 = info.get(l).map_or("", |line| &line.2)
        )
    }

    pfetch_str
}
