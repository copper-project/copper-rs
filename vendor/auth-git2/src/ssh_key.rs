use std::path::Path;

use crate::base64_decode;

/// An error that can occur when analyzing SSH keys.
#[derive(Debug)]
pub enum Error {
	/// Failed to open the key file.
	OpenFile(std::io::Error),

	/// Failed to read from the key file.
	ReadFile(std::io::Error),

	/// Missing PEM trailer in the file (there was a PEM header).
	MissingPemTrailer,

	/// The key is not valid somehow.
	MalformedKey,

	/// There was an invalid base64 blob in the key.
	Base64(base64_decode::Error),
}

/// The format of a key file.
#[derive(Copy, Clone, Eq, PartialEq)]
pub enum KeyFormat {
	/// We don't know what format it is.
	Unknown,

	/// It's an openssh-key-v1 file.
	///
	/// See https://coolaj86.com/articles/the-openssh-private-key-format/ for a description of the format.
	OpensshKeyV1,
}

/// Information about a key file.
pub struct KeyInfo {
	/// The format of the key file.
	pub format: KeyFormat,

	/// Is the key encrypted?
	pub encrypted: bool,
}

/// Analyze an SSH key file.
pub fn analyze_ssh_key_file(priv_key_path: &Path) -> Result<KeyInfo, Error> {
	use std::io::Read;

	let mut buffer = Vec::new();
	let mut file = std::fs::File::open(priv_key_path)
		.map_err(Error::OpenFile)?;
	file.read_to_end(&mut buffer)
		.map_err(Error::ReadFile)?;
	analyze_pem_openssh_key(&buffer)
}

/// Analyze a PEM encoded openssh-key-v1 file.
fn analyze_pem_openssh_key(data: &[u8]) -> Result<KeyInfo, Error> {
	let data = trim_bytes(data);
	let data = match data.strip_prefix(b"-----BEGIN OPENSSH PRIVATE KEY-----") {
		Some(x) => x,
		None => return Ok(KeyInfo { format: KeyFormat::Unknown, encrypted: false }),
	};
	let data = match data.strip_suffix(b"-----END OPENSSH PRIVATE KEY-----") {
		Some(x) => x,
		None => return Err(Error::MissingPemTrailer),
	};
	let data = base64_decode::base64_decode(data).map_err(Error::Base64)?;
	analyze_binary_openssh_key(&data)
}

/// Analyze a binary openss-key-v1 blob.
fn analyze_binary_openssh_key(data: &[u8]) -> Result<KeyInfo, Error> {
	let tail = data.strip_prefix(b"openssh-key-v1\0")
		.ok_or(Error::MalformedKey)?;
	if tail.len() <= 4 {
		return Err(Error::MalformedKey);
	}

	let (cipher_len, tail) = tail.split_at(4);
	let cipher_len = u32::from_be_bytes(cipher_len.try_into().unwrap()) as usize;
	if tail.len() < cipher_len {
		return Err(Error::MalformedKey);
	}
	let cipher = &tail[..cipher_len];
	let encrypted = cipher != b"none";
	Ok(KeyInfo { format: KeyFormat::OpensshKeyV1, encrypted })
}

/// Trim whitespace from the start and end of a byte slice.
fn trim_bytes(data: &[u8]) -> &[u8] {
	let data = match data.iter().position(|b| !b.is_ascii_whitespace()) {
		Some(x) => &data[x..],
		None => return b"",
	};
	let data = match data.iter().rposition(|b| !b.is_ascii_whitespace()) {
		Some(x) => &data[..=x],
		None => return b"",
	};
	data
}

impl std::fmt::Display for Error {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			Self::OpenFile(e) => write!(f, "Failed to open file: {e}"),
			Self::ReadFile(e) => write!(f, "Failed to read from file: {e}"),
			Self::MissingPemTrailer => write!(f, "Missing PEM trailer in key file"),
			Self::MalformedKey => write!(f, "Invalid or malformed key file"),
			Self::Base64(e) => write!(f, "Invalid base64 in key file: {e}"),
		}
	}
}

#[cfg(test)]
mod test {
	use super::*;
	use assert2::assert;

	#[test]
	fn test_is_encrypted_pem_openssh_key() {
		// Encrypted OpenSSH key.
		assert!(let Ok(KeyInfo { format: KeyFormat::OpensshKeyV1, encrypted: true }) = analyze_pem_openssh_key(concat!(
			"-----BEGIN OPENSSH PRIVATE KEY-----\n",
			"b3BlbnNzaC1rZXktdjEAAAAACmFlczI1Ni1jdHIAAAAGYmNyeXB0AAAAGAAAABBddrJWnj\n",
			"6eysG+DqTberHEAAAAEAAAAAEAAAAzAAAAC3NzaC1lZDI1NTE5AAAAIARNG0xAyCq6/OFQ\n",
			"8eQFG1zKYlhtLLz2GC3Sou+C9PTmAAAAoGPGz6ZQhBk8FL4MRDaGsaZuVkPAn/+curIR7r\n",
			"rDoXPAf0/7S2dVWY0gUjolhwlqGFnps4NgukXtKNs4qlAJiVAY/kKPr0fN+ZScuNuKP/Im\n",
			"JbFoNPRaakzgbBwj9/UTpwNgUJa+3fu25l1RMLlrx7OjkQKAHBb6VMsGqH8k9rAEsCCBUK\n",
			"XVJQOMAfa214eo9wgHD06ZnIlk3jS++3hzyUs=\n",
			"-----END OPENSSH PRIVATE KEY-----\n",
		).as_bytes()));

		// Encrypted OpenSSH key with extra random whitespace.
		assert!(let Ok(KeyInfo { format: KeyFormat::OpensshKeyV1, encrypted: true }) = analyze_pem_openssh_key(concat!(
			"   \n\t\r-----BEGIN OPENSSH PRIVATE KEY-----\n",
			"b3BlbnNzaC1rZXktdjEAAAAACmFlczI1Ni1jdHIAAAAGYmNyeXB0AAAAGAAAABBddrJWnj\n",
			"6eysG+DqTberHEAAAAEAAAAAEAAAAzAAAAC3NzaC1lZDI1NTE5AAAAIARNG0xAyCq6/OFQ\n  \r",
			"8eQFG1zKYlhtLLz2GC3Sou+ C9PTmAAAAoGPGz6ZQhBk8FL4MRDaGsaZuVkPAn/+curIR7r\n",
			"rDoXPAf0/7S2dVWY0gUjolhwlqGFnps4NgukXtKNs4qlAJiVAY/kKPr0fN+ZScuNuKP/Im\n",
			"JbFoNPRaakzgbBwj9/UTpwNgUJa+3fu25l1RMLlrx7OjkQKAHBb6VMsGqH8k9rAEsCCBUK\n",
			"XVJQOMAfa214eo9wgHD06ZnIlk3jS++3hzyUs=\n",
			"-----END OPENSSH PRIVATE KEY-----",
		).as_bytes()));

		// Unencrypted OpenSSH key.
		assert!(let Ok(KeyInfo { format: KeyFormat::OpensshKeyV1, encrypted: false }) = analyze_pem_openssh_key(concat!(
			"-----BEGIN OPENSSH PRIVATE KEY-----\n",
			"b3BlbnNzaC1rZXktdjEAAAAABG5vbmUAAAAEbm9uZQAAAAAAAAABAAAAMwAAAAtzc2gtZW\n",
			"QyNTUxOQAAACDTKM0+RYzELoLewv5n5UoEPhmCpwkrtXM4GpWUVF+w3AAAAJhSNRa9UjUW\n",
			"vQAAAAtzc2gtZWQyNTUxOQAAACDTKM0+RYzELoLewv5n5UoEPhmCpwkrtXM4GpWUVF+w3A\n",
			"AAAECZObXz1xTSvl4vpLsMVTuhjroyDteKlW+Uun0yIMl7edMozT5FjMQugt7C/mflSgQ+\n",
			"GYKnCSu1czgalZRUX7DcAAAAEW1hYXJ0ZW5AbWFnbmV0cm9uAQIDBA==\n",
			"-----END OPENSSH PRIVATE KEY-----\n",
		).as_bytes()));
	}
}
