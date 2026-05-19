//! Easy authentication for [`git2`].
//!
//! Authentication with [`git2`] can be quite difficult to implement correctly.
//! This crate aims to make it easy.
//!
//! # Features
//!
//! * Has a small dependency tree.
//! * Can query the SSH agent for private key authentication.
//! * Can get SSH keys from files.
//! * Can prompt the user for passwords for encrypted SSH keys.
//!     * Only supported for OpenSSH private keys.
//! * Can query the git credential helper for usernames and passwords.
//! * Can use pre-provided plain usernames and passwords.
//! * Can prompt the user for credentials as a last resort.
//! * Allows you to fully customize all user prompts.
//!
//! The default user prompts will:
//! * Use the git `askpass` helper if it is configured.
//! * Fall back to prompting the user on the terminal if there is no `askpass` program configured.
//! * Skip the prompt if there is also no terminal available for the process.
//!
//! # Creating an authenticator and enabling authentication mechanisms
//!
//! You can create use [`GitAuthenticator::new()`] (or [`default()`][`GitAuthenticator::default()`]) to create a ready-to-use authenticator.
//! Using one of these constructors will enable all supported authentication mechanisms.
//! You can still add more private key files from non-default locations to try if desired.
//!
//! You can also use [`GitAuthenticator::new_empty()`] to create an authenticator without any authentication mechanism enabled.
//! Then you can selectively enable authentication mechanisms and add custom private key files.
//!
//! # Using the authenticator
//!
//! For the most flexibility, you can get a [`git2::Credentials`] callback using the [`GitAuthenticator::credentials()`] function.
//! You can use it with any git operation that requires authentication.
//! Doing this gives you full control to set other options and callbacks for the git operation.
//!
//! If you don't need to set other options or callbacks, you can also use the convenience functions on [`GitAuthenticator`].
//! They wrap git operations with the credentials callback set:
//!
//! * [`GitAuthenticator::clone_repo()`]
//! * [`GitAuthenticator::fetch()`]
//! * [`GitAuthenticator::download()`]
//! * [`GitAuthenticator::push()`]
//!
//! # Customizing user prompts
//!
//! All user prompts can be fully customized by calling [`GitAuthenticator::set_prompter()`].
//! This allows you to override the way that the user is prompted for credentials or passphrases.
//!
//! If you have a fancy user interface, you can use a custom prompter to integrate the prompts with your user interface.
//!
//! # Example: Clone a repository
//!
//! ```no_run
//! # fn main() -> Result<(), git2::Error> {
//! use auth_git2::GitAuthenticator;
//! use std::path::Path;
//!
//! let url = "https://github.com/de-vri-es/auth-git2-rs";
//! let into = Path::new("/tmp/dyfhxoaj/auth-git2-rs");
//!
//! let auth = GitAuthenticator::default();
//! let mut repo = auth.clone_repo(url, into);
//! # let _ = repo;
//! # Ok(())
//! # }
//! ```
//!
//! # Example: Clone a repository with full control over fetch options
//!
//! ```no_run
//! # fn main() -> Result<(), git2::Error> {
//! use auth_git2::GitAuthenticator;
//! use std::path::Path;
//!
//! let auth = GitAuthenticator::default();
//! let git_config = git2::Config::open_default()?;
//! let mut repo_builder = git2::build::RepoBuilder::new();
//! let mut fetch_options = git2::FetchOptions::new();
//! let mut remote_callbacks = git2::RemoteCallbacks::new();
//!
//! remote_callbacks.credentials(auth.credentials(&git_config));
//! fetch_options.remote_callbacks(remote_callbacks);
//! repo_builder.fetch_options(fetch_options);
//!
//! let url = "https://github.com/de-vri-es/auth-git2-rs";
//! let into = Path::new("/tmp/dyfhxoaj/auth-git2-rs");
//! let mut repo = repo_builder.clone(url, into);
//! # let _ = repo;
//! # Ok(())
//! # }
//! ```

#![warn(missing_docs)]

use std::collections::BTreeMap;
use std::path::{PathBuf, Path};

#[cfg(feature = "log")]
mod log {
	pub use ::log::warn;
	pub use ::log::debug;
	pub use ::log::trace;
}

#[cfg(feature = "log")]
use crate::log::*;

#[cfg(not(feature = "log"))]
#[macro_use]
mod log {
	macro_rules! warn {
		($($tokens:tt)*) => { { let _ = format_args!($($tokens)*); } };
	}

	macro_rules! debug {
		($($tokens:tt)*) => { { let _ = format_args!($($tokens)*); } };
	}

	macro_rules! trace {
		($($tokens:tt)*) => { { let _ = format_args!($($tokens)*); } };
	}
}

mod base64_decode;
mod default_prompt;
mod prompter;
mod ssh_key;

pub use prompter::Prompter;

/// Configurable authenticator to use with [`git2`].
#[derive(Clone)]
pub struct GitAuthenticator {
	/// Map of domain names to plaintext credentials.
	plaintext_credentials: BTreeMap<String, PlaintextCredentials>,

	/// Try getting username/password from the git credential helper.
	try_cred_helper: bool,

	/// Number of times to ask the user for a username/password on the terminal.
	try_password_prompt: u32,

	/// Map of domain names to usernames to try for SSH connections if no username was specified.
	usernames: BTreeMap<String, String>,

	/// Try to use the SSH agent to get a working SSH key.
	try_ssh_agent: bool,

	/// SSH keys to use from file.
	ssh_keys: Vec<PrivateKeyFile>,

	/// Prompt for passwords for encrypted SSH keys.
	prompt_ssh_key_password: bool,

	/// Custom prompter to use.
	prompter: Box<dyn prompter::ClonePrompter>,
}

impl std::fmt::Debug for GitAuthenticator {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		f.debug_struct("GitAuthenticator")
			.field("plaintext_credentials", &self.plaintext_credentials)
			.field("try_cred_helper", &self.try_cred_helper)
			.field("try_password_prompt", &self.try_password_prompt)
			.field("usernames", &self.usernames)
			.field("try_ssh_agent", &self.try_ssh_agent)
			.field("ssh_keys", &self.ssh_keys)
			.field("prompt_ssh_key_password", &self.prompt_ssh_key_password)
			.finish()
	}
}

impl Default for GitAuthenticator {
	/// Create a new authenticator with all supported options enabled.
	///
	/// This is the same as [`GitAuthenticator::new()`].
	fn default() -> Self {
		Self::new()
	}
}

impl GitAuthenticator {
	/// Create a new authenticator with all supported options enabled.
	///
	/// This is equivalent to:
	/// ```
	/// # use auth_git2::GitAuthenticator;
	/// GitAuthenticator::new_empty()
	///     .try_cred_helper(true)
	///     .try_password_prompt(3)
	///     .add_default_username()
	///     .try_ssh_agent(true)
	///     .add_default_ssh_keys()
	///     .prompt_ssh_key_password(true)
	/// # ;
	/// ```
	pub fn new() -> Self {
		Self::new_empty()
			.try_cred_helper(true)
			.try_password_prompt(3)
			.add_default_username()
			.try_ssh_agent(true)
			.add_default_ssh_keys()
			.prompt_ssh_key_password(true)
	}

	/// Create a new authenticator with all authentication options disabled.
	pub fn new_empty() -> Self {
		Self {
			try_ssh_agent: false,
			try_cred_helper: false,
			plaintext_credentials: BTreeMap::new(),
			try_password_prompt: 0,
			usernames: BTreeMap::new(),
			ssh_keys: Vec::new(),
			prompt_ssh_key_password: false,
			prompter: prompter::wrap_prompter(default_prompt::DefaultPrompter),
		}
	}

	/// Set the username + password to use for a specific domain.
	///
	/// Use the special value "*" for the domain name to add fallback credentials when there is no exact match for the domain.
	pub fn add_plaintext_credentials(mut self, domain: impl Into<String>, username: impl Into<String>, password: impl Into<String>) -> Self {
		let domain = domain.into();
		let username = username.into();
		let password = password.into();
		self.plaintext_credentials.insert(domain, PlaintextCredentials {
			username,
			password,
		});
		self
	}

	/// Configure if the git credentials helper should be used.
	///
	/// See the git documentation of the `credential.helper` configuration options for more details.
	pub fn try_cred_helper(mut self, enable: bool) -> Self {
		self.try_cred_helper = enable;
		self
	}

	/// Configure the number of times we should prompt the user for a username/password.
	///
	/// Setting this value to `0` disables password prompts.
	///
	/// By default, if an `askpass` helper is configured, it will be used for the prompts.
	/// Otherwise, the user will be prompted directly on the terminal of the current process.
	/// If there is also no terminal available, the prompt is skipped.
	///
	/// An `askpass` helper can be configured in the `GIT_ASKPASS` environment variable,
	/// the `core.askPass` configuration value or the `SSH_ASKPASS` environment variable.
	///
	/// You can override the prompt behaviour by calling [`Self::set_prompter()`].
	pub fn try_password_prompt(mut self, max_count: u32) -> Self {
		self.try_password_prompt = max_count;
		self
	}

	/// Use a custom [`Prompter`] to prompt the user for credentials and passphrases.
	///
	/// If you set a custom prompter,
	/// the authenticator will no longer try to use the `askpass` helper or prompt the user on the terminal.
	/// Instead, the provided prompter will be called.
	///
	/// Note that prompts must still be enabled with [`Self::try_password_prompt()`] and [`Self::prompt_ssh_key_password()`].
	/// If prompts are disabled, your custom prompter will not be called.
	///
	/// You can use this function to integrate the prompts with your own user interface
	/// or simply to tweak the way the user is prompted on the terminal.
	///
	/// A unique clone of the prompter will be used for each [`git2::Credentials`] callback returned by [`Self::credentials()`].
	pub fn set_prompter<P: Prompter + Clone + Send + 'static>(mut self, prompter: P) -> Self {
		self.prompter = prompter::wrap_prompter(prompter);
		self
	}

	/// Add a username to try for authentication for a specific domain.
	///
	/// Some authentication mechanisms need a username, but not all valid git URLs specify one.
	/// You can add one or more usernames to try in that situation.
	///
	/// You can use the special domain name "*" to set a fallback username for domains that do not have a specific username set.
	pub fn add_username(mut self, domain: impl Into<String>, username: impl Into<String>) -> Self {
		let domain = domain.into();
		let username = username.into();
		self.usernames.insert(domain, username);
		self
	}

	/// Add the default username to try.
	///
	/// The default username if read from the `USER` or `USERNAME` environment variable.
	pub fn add_default_username(self) -> Self {
		if let Ok(username) = std::env::var("USER").or_else(|_| std::env::var("USERNAME")) {
			self.add_username("*", username)
		} else {
			self
		}
	}

	/// Configure if the SSH agent should be used for public key authentication.
	pub fn try_ssh_agent(mut self, enable: bool) -> Self {
		self.try_ssh_agent = enable;
		self
	}

	/// Add a private key to use for public key authentication.
	///
	/// The key will be read from disk by `git2`, so it must still exist when the authentication is performed.
	///
	/// You can provide a password for decryption of the private key.
	/// If no password is provided and the `Self::prompt_ssh_key_password()` is enabled,
	/// the user will be prompted for the passphrase of encrypted keys.
	/// Note that currently only the `OpenSSH` private key format is supported for detecting that a key is encrypted.
	///
	/// A matching `.pub` file will also be read if it exists.
	/// For example, if you add the private key `"foo/my_ssh_id"`,
	/// then `"foo/my_ssh_id.pub"` will be used too, if it exists.
	pub fn add_ssh_key_from_file(mut self, private_key: impl Into<PathBuf>, password: impl Into<Option<String>>) -> Self {
		let private_key = private_key.into();
		let public_key = get_pub_key_path(&private_key);
		let password = password.into();
		self.ssh_keys.push(PrivateKeyFile {
			private_key,
			public_key,
			password,
		});
		self
	}

	/// Add all default SSH keys for public key authentication.
	///
	/// This will add all of the following files, if they exist:
	///
	/// * `"$HOME/.ssh/id_rsa"`
	/// * `"$HOME/.ssh/id_ecdsa"`
	/// * `"$HOME/.ssh/id_ecdsa_sk"`
	/// * `"$HOME/.ssh/id_ed25519"`
	/// * `"$HOME/.ssh/id_ed25519_sk"`
	/// * `"$HOME/.ssh/id_dsa"`
	pub fn add_default_ssh_keys(mut self) -> Self {
		let ssh_dir = match dirs::home_dir() {
			Some(x) => x.join(".ssh"),
			None => return self,
		};

		let candidates = [
			"id_rsa",
			"id_ecdsa",
			"id_ecdsa_sk",
			"id_ed25519",
			"id_ed25519_sk",
			"id_dsa",
		];

		for candidate in candidates {
			let private_key = ssh_dir.join(candidate);
			if !private_key.is_file() {
				continue;
			}
			self = self.add_ssh_key_from_file(private_key, None);
		}

		self
	}

	/// Prompt for passwords for encrypted SSH keys if needed.
	///
	/// By default, if an `askpass` helper is configured, it will be used for the prompts.
	/// Otherwise, the user will be prompted directly on the terminal of the current process.
	/// If there is also no terminal available, the prompt is skipped.
	///
	/// An `askpass` helper can be configured in the `GIT_ASKPASS` environment variable,
	/// the `core.askPass` configuration value or the `SSH_ASKPASS` environment variable.
	///
	/// You can override the prompt behaviour by calling [`Self::set_prompter()`].
	pub fn prompt_ssh_key_password(mut self, enable: bool) -> Self {
		self.prompt_ssh_key_password = enable;
		self
	}

	/// Get the credentials callback to use for [`git2::Credentials`].
	///
	/// # Example: Fetch from a remote with authentication
	/// ```no_run
	/// # fn foo(repo: &mut git2::Repository) -> Result<(), git2::Error> {
	/// use auth_git2::GitAuthenticator;
	///
	/// let auth = GitAuthenticator::default();
	/// let git_config = repo.config()?;
	/// let mut fetch_options = git2::FetchOptions::new();
	/// let mut remote_callbacks = git2::RemoteCallbacks::new();
	///
	/// remote_callbacks.credentials(auth.credentials(&git_config));
	/// fetch_options.remote_callbacks(remote_callbacks);
	///
	/// repo.find_remote("origin")?
	///     .fetch(&["main"], Some(&mut fetch_options), None)?;
	/// # Ok(())
	/// # }
	/// ```
	pub fn credentials<'a>(
		&'a self,
		git_config: &'a git2::Config,
	) -> impl 'a + FnMut(&str, Option<&str>, git2::CredentialType) -> Result<git2::Cred, git2::Error> {
		make_credentials_callback(self, git_config)
	}

	/// Clone a repository using the git authenticator.
	///
	/// If you need more control over the clone options,
	/// use [`Self::credentials()`] with a [`git2::build::RepoBuilder`].
	pub fn clone_repo(&self, url: impl AsRef<str>, into: impl AsRef<Path>) -> Result<git2::Repository, git2::Error> {
		let url = url.as_ref();
		let into = into.as_ref();

		let git_config = git2::Config::open_default()?;
		let mut repo_builder = git2::build::RepoBuilder::new();
		let mut fetch_options = git2::FetchOptions::new();
		let mut remote_callbacks = git2::RemoteCallbacks::new();

		remote_callbacks.credentials(self.credentials(&git_config));
		fetch_options.remote_callbacks(remote_callbacks);
		repo_builder.fetch_options(fetch_options);

		repo_builder.clone(url, into)
	}


	/// Fetch from a remote using the git authenticator.
	///
	/// If you need more control over the fetch options,
	/// use [`Self::credentials()`] with [`git2::Remote::fetch()`].
	pub fn fetch(&self, repo: &git2::Repository, remote: &mut git2::Remote, refspecs: &[&str], reflog_msg: Option<&str>) -> Result<(), git2::Error> {
		let git_config = repo.config()?;
		let mut fetch_options = git2::FetchOptions::new();
		let mut remote_callbacks = git2::RemoteCallbacks::new();

		remote_callbacks.credentials(self.credentials(&git_config));
		fetch_options.remote_callbacks(remote_callbacks);
		remote.fetch(refspecs, Some(&mut fetch_options), reflog_msg)
	}

	/// Download and index the packfile from a remote using the git authenticator.
	///
	/// If you need more control over the download options,
	/// use [`Self::credentials()`] with [`git2::Remote::download()`].
	///
	/// This function does not update the remote tracking branches.
	/// Consider using [`Self::fetch()`] if that is what you want.
	pub fn download(&self, repo: &git2::Repository, remote: &mut git2::Remote, refspecs: &[&str]) -> Result<(), git2::Error> {
		let git_config = repo.config()?;
		let mut fetch_options = git2::FetchOptions::new();
		let mut remote_callbacks = git2::RemoteCallbacks::new();

		remote_callbacks.credentials(self.credentials(&git_config));
		fetch_options.remote_callbacks(remote_callbacks);
		remote.download(refspecs, Some(&mut fetch_options))
	}

	/// Push to a remote using the git authenticator.
	///
	/// If you need more control over the push options,
	/// use [`Self::credentials()`] with [`git2::Remote::push()`].
	pub fn push(&self, repo: &git2::Repository, remote: &mut git2::Remote, refspecs: &[&str]) -> Result<(), git2::Error> {
		let git_config = repo.config()?;
		let mut push_options = git2::PushOptions::new();
		let mut remote_callbacks = git2::RemoteCallbacks::new();

		remote_callbacks.credentials(self.credentials(&git_config));
		push_options.remote_callbacks(remote_callbacks);

		remote.push(refspecs, Some(&mut push_options))
	}

	/// Get the configured username for a URL.
	fn get_username(&self, url: &str) -> Option<&str> {
		if let Some(domain) = domain_from_url(url) {
			if let Some(username) = self.usernames.get(domain) {
				return Some(username);
			}
		}
		self.usernames.get("*").map(|x| x.as_str())
	}

	/// Get the configured plaintext credentials for a URL.
	fn get_plaintext_credentials(&self, url: &str) -> Option<&PlaintextCredentials> {
		if let Some(domain) = domain_from_url(url) {
			if let Some(credentials) = self.plaintext_credentials.get(domain) {
				return Some(credentials);
			}
		}
		self.plaintext_credentials.get("*")
	}
}

fn make_credentials_callback<'a>(
	authenticator: &'a GitAuthenticator,
	git_config: &'a git2::Config,
) -> impl 'a + FnMut(&str, Option<&str>, git2::CredentialType) -> Result<git2::Cred, git2::Error> {
	let mut try_cred_helper = authenticator.try_cred_helper;
	let mut try_password_prompt = authenticator.try_password_prompt;
	let mut try_ssh_agent = authenticator.try_ssh_agent;
	let mut ssh_keys = authenticator.ssh_keys.iter();
	let mut prompter = authenticator.prompter.clone();

	move |url: &str, username: Option<&str>, allowed: git2::CredentialType| {
		trace!("credentials callback called with url: {url:?}, username: {username:?}, allowed_credentials: {allowed:?}");

		// If git2 is asking for a username, we got an SSH url without username specified.
		// After we supply a username, it will ask for the real credentials.
		//
		// Sadly, we can not switch usernames during an authentication session,
		// so to try different usernames, we need to retry the git operation multiple times.
		// If this happens, we'll bail and go into stage 2.
		if allowed.contains(git2::CredentialType::USERNAME) {
			if let Some(username) = authenticator.get_username(url) {
				debug!("credentials_callback: returning username: {username:?}");
				match git2::Cred::username(username) {
					Ok(x) => return Ok(x),
					Err(e) => {
						debug!("credentials_callback: failed to wrap username: {e}");
						return Err(e);
					},
				}
			}
		}

		// Try public key authentication.
		if allowed.contains(git2::CredentialType::SSH_KEY) {
			if let Some(username) = username {
				if try_ssh_agent {
					try_ssh_agent = false;
					debug!("credentials_callback: trying ssh_key_from_agent with username: {username:?}");
					match git2::Cred::ssh_key_from_agent(username) {
						Ok(x) => return Ok(x),
						Err(e) => debug!("credentials_callback: failed to use SSH agent: {e}"),
					}
				}

				#[allow(clippy::while_let_on_iterator)] // Incorrect lint: we're not consuming the iterator.
				while let Some(key) = ssh_keys.next() {
					debug!("credentials_callback: trying ssh key, username: {username:?}, private key: {:?}", key.private_key);
					let prompter = Some(prompter.as_prompter_mut())
						.filter(|_| authenticator.prompt_ssh_key_password);
					match key.to_credentials(username, prompter, git_config) {
						Ok(x) => return Ok(x),
						Err(e) => debug!("credentials_callback: failed to use SSH key from file {:?}: {e}", key.private_key),
					}
				}
			}
		}

		// Sometimes libgit2 will ask for a username/password in plaintext.
		if allowed.contains(git2::CredentialType::USER_PASS_PLAINTEXT) {
			// Try provided plaintext credentials first.
			if let Some(credentials) = authenticator.get_plaintext_credentials(url) {
				debug!("credentials_callback: trying plain text credentials with username: {:?}", credentials.username);
				match credentials.to_credentials() {
					Ok(x) => return Ok(x),
					Err(e) => {
						debug!("credentials_callback: failed to wrap plain text credentials: {e}");
						return Err(e);
					},
				}
			}

			// Try the git credential helper.
			if try_cred_helper {
				try_cred_helper = false;
				debug!("credentials_callback: trying credential_helper");
				match git2::Cred::credential_helper(git_config, url, username) {
					Ok(x) => return Ok(x),
					Err(e) => debug!("credentials_callback: failed to use credential helper: {e}"),
				}
			}

			// Prompt the user on the terminal.
			if try_password_prompt > 0 {
				try_password_prompt -= 1;
				let credentials = PlaintextCredentials::prompt(
					prompter.as_prompter_mut(),
					username,
					url,
					git_config
				);
				if let Some(credentials) = credentials {
					return credentials.to_credentials();
				}
			}
		}

		Err(git2::Error::from_str("all authentication attempts failed"))
	}
}

#[derive(Debug, Clone)]
struct PrivateKeyFile {
	private_key: PathBuf,
	public_key: Option<PathBuf>,
	password: Option<String>,
}

impl PrivateKeyFile {
	fn to_credentials(&self, username: &str, prompter: Option<&mut dyn Prompter>, git_config: &git2::Config) -> Result<git2::Cred, git2::Error> {
		if let Some(password) = &self.password {
			git2::Cred::ssh_key(username, self.public_key.as_deref(), &self.private_key, Some(password))
		} else if let Some(prompter) = prompter {
			let password = match ssh_key::analyze_ssh_key_file(&self.private_key) {
				Err(e) => {
					warn!("Failed to analyze SSH key: {}: {}", self.private_key.display(), e);
					None
				},
				Ok(key_info) => {
					if key_info.format == ssh_key::KeyFormat::Unknown {
						warn!("Unknown key format for key: {}", self.private_key.display());
					}
					if key_info.encrypted {
						prompter.prompt_ssh_key_passphrase(&self.private_key, git_config)
					} else {
						None
					}
				},
			};
			git2::Cred::ssh_key(username, self.public_key.as_deref(), &self.private_key, password.as_deref())
		} else {
			git2::Cred::ssh_key(username, self.public_key.as_deref(), &self.private_key, None)
		}
	}
}

#[derive(Debug, Clone)]
struct PlaintextCredentials {
	username: String,
	password: String,
}

impl PlaintextCredentials {
	fn prompt(prompter: &mut dyn Prompter, username: Option<&str>, url: &str, git_config: &git2::Config) -> Option<Self> {
		if let Some(username) = username {
			let password = prompter.prompt_password(username, url, git_config)?;
			Some(Self {
				username: username.into(),
				password,
			})
		} else {
			let (username, password) = prompter.prompt_username_password(url, git_config)?;
			Some(Self {
				username,
				password,
			})
		}
	}

	fn to_credentials(&self) -> Result<git2::Cred, git2::Error> {
		git2::Cred::userpass_plaintext(&self.username, &self.password)
	}
}

fn get_pub_key_path(priv_key_path: &Path) -> Option<PathBuf> {
	let name = priv_key_path.file_name()?;
	let name = name.to_str()?;
	let pub_key_path = priv_key_path.with_file_name(format!("{name}.pub"));
	if pub_key_path.is_file() {
		Some(pub_key_path)
	} else {
		None
	}
}

fn domain_from_url(url: &str) -> Option<&str> {
	// We support:
	// Relative paths
	// Real URLs: scheme://[user[:pass]@]host/path
	// SSH URLs: [user@]host:path.

	// If there is no colon: URL is a relative path and there is no domain (or need for credentials).
	let (head, tail) = url.split_once(':')?;

	// Real URL
	if let Some(tail) = tail.strip_prefix("//") {
		let (_credentials, tail) = tail.split_once('@').unwrap_or(("", tail));
		let (host, _path) = tail.split_once('/').unwrap_or((tail, ""));
		Some(host)
	// SSH "URL"
	} else {
		let (_credentials, host) = head.split_once('@').unwrap_or(("", head));
		Some(host)
	}
}

#[cfg(test)]
mod test {
	use super::*;
	use assert2::assert;

	#[test]
	fn test_domain_from_url() {
		assert!(let Some("host") = domain_from_url("user@host:path"));
		assert!(let Some("host") = domain_from_url("host:path"));
		assert!(let Some("host") = domain_from_url("host:path@with:stuff"));

		assert!(let Some("host") = domain_from_url("ssh://user:pass@host/path"));
		assert!(let Some("host") = domain_from_url("ssh://user@host/path"));
		assert!(let Some("host") = domain_from_url("ssh://host/path"));

		assert!(let None = domain_from_url("some/relative/path"));
		assert!(let None = domain_from_url("some/relative/path@with-at-sign"));
	}

	#[test]
	fn test_that_authenticator_is_send() {
		let authenticator = GitAuthenticator::new();
		let thread = std::thread::spawn(move || {
			drop(authenticator);
		});
		thread.join().unwrap();
	}
}
