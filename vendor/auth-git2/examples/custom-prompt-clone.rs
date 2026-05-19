use std::path::{Path, PathBuf};

#[derive(Copy, Clone)]
struct YadPrompter;

impl auth_git2::Prompter for YadPrompter {
	fn prompt_username_password(&mut self, url: &str, _git_config: &git2::Config) -> Option<(String, String)> {
		let mut items = yad_prompt(
			"Git authentication",
			&format!("Authentication required for {url}"),
			&["Username", "Password:H"],
		).ok()?.into_iter();
		let username = items.next()?;
		let password = items.next()?;
		Some((username, password))
	}

	fn prompt_password(&mut self, username: &str, url: &str, _git_config: &git2::Config) -> Option<String> {
		let mut items = yad_prompt(
			"Git authentication",
			&format!("Authentication required for {url}"),
			&[&format!("Username: {username}:LBL"), "Password:H"],
		).ok()?.into_iter();
		let password = items.next()?;
		Some(password)
	}

	fn prompt_ssh_key_passphrase(&mut self, private_key_path: &std::path::Path, _git_config: &git2::Config) -> Option<String> {
		let mut items = yad_prompt(
			"Git authentication",
			&format!("Passphrase required for {}", private_key_path.display()),
			&["Passphrase:H"],
		).ok()?.into_iter();
		let passphrase = items.next()?;
		Some(passphrase)
	}
}

fn yad_prompt(title: &str, text: &str, fields: &[&str]) -> Result<Vec<String>, ()> {
	let mut command = std::process::Command::new("yad");
	command
		.arg("--title")
		.arg(title)
		.arg("--text")
		.arg(text)
		.arg("--form")
		.arg("--separator=\n");
	for field in fields {
		command.arg("--field");
		command.arg(field);
	}

	let output = command
		.stderr(std::process::Stdio::inherit())
		.output()
		.map_err(|e| log::error!("Failed to run `yad`: {e}"))?;

	if !output.status.success() {
		log::debug!("yad exited with {}", output.status);
		return Err(());
	}

	let output = String::from_utf8(output.stdout)
		.map_err(|_| log::warn!("Invalid UTF-8 in response from yad"))?;

	let mut items: Vec<_> = output.splitn(fields.len() + 1, '\n')
		.take(fields.len())
		.map(|x| x.to_owned())
		.collect();
	if let Some(last) = items.pop() {
		if !last.is_empty() {
			items.push(last)
		}
	}

	if items.len() != fields.len() {
		log::error!("asked yad for {} values but got only {}", fields.len(), items.len());
		Err(())
	} else {
		Ok(items)
	}
}

#[derive(clap::Parser)]
struct Options {
	/// Show more verbose statement.
	#[clap(long, short)]
	#[clap(global = true)]
	#[clap(action = clap::ArgAction::Count)]
	verbose: u8,

	/// The URL of the repository to clone.
	#[clap(value_name = "URL")]
	repo: String,

	/// The path where to clone the repository.
	#[clap(value_name = "PATH")]
	local_path: Option<PathBuf>,
}

fn main() {
	if let Err(()) = do_main(clap::Parser::parse()) {
		std::process::exit(1);
	}
}

fn log_level(verbose: u8) -> log::LevelFilter {
	match verbose {
		0 => log::LevelFilter::Info,
		1 => log::LevelFilter::Debug,
		2.. => log::LevelFilter::Trace,
	}
}

fn do_main(options: Options) -> Result<(), ()> {
	let log_level = log_level(options.verbose);
	env_logger::builder()
		.parse_default_env()
		.filter_module(module_path!(), log_level)
		.filter_module("auth_git2", log_level)
		.init();

	let local_path = options.local_path.as_deref()
		.unwrap_or_else(|| Path::new(repo_name_from_url(&options.repo)));

	log::info!("Cloning {} into {}", options.repo, local_path.display());

	let auth = auth_git2::GitAuthenticator::default()
		.set_prompter(YadPrompter);
	auth.clone_repo(&options.repo, local_path)
		.map_err(|e| log::error!("Failed to clone {}: {}", options.repo, e))?;
	Ok(())
}

fn repo_name_from_url(url: &str) -> &str {
	url.rsplit_once('/')
		.map(|(_head, tail)| tail)
		.unwrap_or(url)
}
