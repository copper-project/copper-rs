use std::io::Write;
use std::path::{Path, PathBuf};

#[cfg(feature = "log")]
use crate::log::*;

#[derive(Copy, Clone)]
pub(crate) struct DefaultPrompter;

impl crate::Prompter for DefaultPrompter {
	fn prompt_username_password(&mut self, url: &str, git_config: &git2::Config) -> Option<(String, String)> {
		prompt_username_password(url, git_config)
			.map_err(|e| log_error("username and password", &e))
			.ok()
	}

	fn prompt_password(&mut self, username: &str, url: &str, git_config: &git2::Config) -> Option<String> {
		prompt_password(username, url, git_config)
			.map_err(|e| log_error("password", &e))
			.ok()
	}

	fn prompt_ssh_key_passphrase(&mut self, private_key_path: &Path, git_config: &git2::Config) -> Option<String> {
		prompt_ssh_key_passphrase(private_key_path, git_config)
			.map_err(|e| log_error("SSH key passphrase", &e))
			.ok()
	}
}

fn log_error(kind: &str, error: &Error) {
	warn!("Failed to prompt the user for {kind}: {error}");
	if let Error::AskpassExitStatus(error) = error {
		if let Some(extra_message) = error.extra_message() {
			for line in extra_message.lines() {
				warn!("askpass: {line}");
			}
		}
	}
}

/// Error that can occur when prompting for a password.
pub enum Error {
	/// Failed to run the askpass command.
	AskpassCommand(std::io::Error),

	/// Askpass command exitted with a non-zero error code.
	AskpassExitStatus(AskpassExitStatusError),

	/// Password contains invalid UTF-8.
	InvalidUtf8,

	/// Failed to open a handle to the main terminal of the process.
	OpenTerminal(std::io::Error),

	/// Failed to read/write to the terminal.
	ReadWriteTerminal(std::io::Error),
}

/// The askpass process exited with a non-zero exit code.
pub struct AskpassExitStatusError {
	/// The exit status of the askpass process.
	pub status: std::process::ExitStatus,

	/// The standard error of the askpass process.
	pub stderr: Result<String, std::string::FromUtf8Error>,
}

impl AskpassExitStatusError {
	/// Get the extra error message, if any.
	///
	/// This will give the standard error of the askpass process if it exited with an error.
	pub fn extra_message(&self) -> Option<&str> {
		self.stderr.as_deref().ok()
	}
}

/// Prompt the user for a username and password for a particular URL.
///
/// This uses the askpass helper if configured,
/// and falls back to prompting on the terminal otherwise.
fn prompt_username_password(url: &str, git_config: &git2::Config) -> Result<(String, String), Error> {
	if let Some(askpass) = askpass_command(git_config) {
		let username = askpass_prompt(&askpass, &format!("Username for {url}"))?;
		let password = askpass_prompt(&askpass, &format!("Password for {url}"))?;
		Ok((username, password))
	} else {
		let mut terminal = terminal_prompt::Terminal::open()
			.map_err(Error::OpenTerminal)?;
		writeln!(terminal, "Authentication needed for {url}")
			.map_err(Error::ReadWriteTerminal)?;
		let username = terminal.prompt("Username: ")
			.map_err(Error::ReadWriteTerminal)?;
		let password = terminal.prompt_sensitive("Password: ")
			.map_err(Error::ReadWriteTerminal)?;
		Ok((username, password))
	}
}

/// Prompt the user for a password for a particular URL and username.
///
/// This uses the askpass helper if configured,
/// and falls back to prompting on the terminal otherwise.
fn prompt_password(_username: &str, url: &str, git_config: &git2::Config) -> Result<String, Error> {
	if let Some(askpass) = askpass_command(git_config) {
		let password = askpass_prompt(&askpass, &format!("Password for {url}"))?;
		Ok(password)
	} else {
		let mut terminal = terminal_prompt::Terminal::open()
			.map_err(Error::OpenTerminal)?;
		writeln!(terminal, "Authentication needed for {url}")
			.map_err(Error::ReadWriteTerminal)?;
		let password = terminal.prompt_sensitive("Password: ")
			.map_err(Error::ReadWriteTerminal)?;
		Ok(password)
	}
}

/// Prompt the user for the password of an encrypted SSH key.
///
/// This uses the askpass helper if configured,
/// and falls back to prompting on the terminal otherwise.
fn prompt_ssh_key_passphrase(private_key_path: &Path, git_config: &git2::Config) -> Result<String, Error> {
	if let Some(askpass) = askpass_command(git_config) {
		askpass_prompt(&askpass, &format!("Password for {}", private_key_path.display()))
	} else {
		let mut terminal = terminal_prompt::Terminal::open()
			.map_err(Error::OpenTerminal)?;
		writeln!(terminal, "Password needed for {}", private_key_path.display())
			.map_err(Error::ReadWriteTerminal)?;
		terminal.prompt_sensitive("Password: ")
			.map_err(Error::ReadWriteTerminal)
	}
}

/// Get the configured askpass program, if any.
fn askpass_command(git_config: &git2::Config) -> Option<PathBuf> {
	if let Some(command) = std::env::var_os("GIT_ASKPASS") {
		Some(command.into())
	} else if let Ok(command) = git_config.get_path("core.askPass") {
		return Some(command)
	} else if let Some(command) = std::env::var_os("SSH_ASKPASS") {
		return Some(command.into());
	} else {
		None
	}
}

/// Prompt the user using the given askpass program.
fn askpass_prompt(program: &Path, prompt: &str) -> Result<String, Error> {
	let output = std::process::Command::new(program)
		.arg(prompt)
		.output()
		.map_err(Error::AskpassCommand)?;
	if output.status.success() {
		let password = String::from_utf8(output.stdout)
			.map_err(|_| Error::InvalidUtf8)?;
		Ok(password)
	} else {
		// Do not keep stdout, it could contain a password D:
		Err(Error::AskpassExitStatus(AskpassExitStatusError {
			status: output.status,
			stderr: String::from_utf8(output.stderr),
		}))
	}
}

impl std::fmt::Display for Error {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			Self::AskpassCommand(e) => write!(f, "Failed to run askpass command: {e}"),
			Self::AskpassExitStatus(e) => write!(f, "{e}"),
			Self::InvalidUtf8 => write!(f, "User response contains invalid UTF-8"),
			Self::OpenTerminal(e) => write!(f, "Failed to open terminal: {e}"),
			Self::ReadWriteTerminal(e) => write!(f, "Failed to read/write to terminal: {e}"),
		}
	}
}

impl std::fmt::Display for AskpassExitStatusError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		write!(f, "Program exitted with {}", self.status)
	}
}
