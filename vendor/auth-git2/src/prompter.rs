use std::path::Path;

/// Trait for customizing user prompts.
///
/// You can provide an implementor of this trait to customize the way a user is prompted for credentials and passphrases.
pub trait Prompter: Send {
	/// Promp the user for a username and password.
	///
	/// If the prompt fails or the user fails to provide the requested information, this function should return `None`.
	fn prompt_username_password(&mut self, url: &str, git_config: &git2::Config) -> Option<(String, String)>;

	/// Promp the user for a password when the username is already known.
	///
	/// If the prompt fails or the user fails to provide the requested information, this function should return `None`.
	fn prompt_password(&mut self, username: &str, url: &str, git_config: &git2::Config) -> Option<String>;

	/// Promp the user for the passphrase of an encrypted SSH key.
	///
	/// If the prompt fails or the user fails to provide the requested information, this function should return `None`.
	fn prompt_ssh_key_passphrase(&mut self, private_key_path: &Path, git_config: &git2::Config) -> Option<String>;
}

/// Wrap a clonable [`Prompter`] in a `Box<dyn MakePrompter>`.
pub(crate) fn wrap_prompter<P>(prompter: P) -> Box<dyn ClonePrompter>
where
	P: Prompter + Clone + 'static,
{
	Box::new(prompter)
}

/// Trait to allow making clones of a `Box<dyn Prompter + Send>`.
pub(crate) trait ClonePrompter: Prompter {
	/// Clone the `Box<dyn ClonePrompter>`.
	fn dyn_clone(&self) -> Box<dyn ClonePrompter>;

	/// Get `self` as plain `Prompter`.
	fn as_prompter_mut(&mut self) -> &mut dyn Prompter;
}

/// Implement `ClonePrompter` for clonable Prompters.
impl<P> ClonePrompter for P
where
	P: Prompter + Clone + 'static,
{
	fn dyn_clone(&self) -> Box<dyn ClonePrompter> {
		Box::new(self.clone())
	}

	fn as_prompter_mut(&mut self) -> &mut dyn Prompter {
		self
	}
}

impl Clone for Box<dyn ClonePrompter> {
	fn clone(&self) -> Self {
		self.dyn_clone()
	}
}
