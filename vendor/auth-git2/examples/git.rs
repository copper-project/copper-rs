use std::path::{Path, PathBuf};

#[derive(clap::Parser)]
struct Options {
	/// Show more verbose statement.
	#[clap(long, short)]
	#[clap(global = true)]
	#[clap(action = clap::ArgAction::Count)]
	verbose: u8,

	/// The subcommand.
	#[clap(subcommand)]
	command: Command,
}

#[derive(clap::Subcommand)]
enum Command {
	Clone(CloneCommand),
	Fetch(FetchCommand),
	Push(PushCommand),
}

/// Clone a repository.
#[derive(clap::Parser)]
struct CloneCommand {
	/// The URL of the repository to clone.
	#[clap(value_name = "URL")]
	repo: String,

	/// The path where to clone the repository.
	#[clap(value_name = "PATH")]
	local_path: Option<PathBuf>,
}

/// Fetch from a remote.
#[derive(clap::Parser)]
struct FetchCommand {
	/// The repository to operate on.
	#[clap(value_name = "PATH")]
	#[clap(short = 'C', long)]
	repo: PathBuf,

	/// The repository to operate on.
	#[clap(value_name = "REMOTE")]
	remote: String,

	/// The refs to fetch.
	#[clap(trailing_var_arg = true)]
	#[clap(required = true)]
	refspec: Vec<String>,
}

/// Push to a remote.
#[derive(clap::Parser)]
struct PushCommand {
	/// The repository to operate on.
	#[clap(value_name = "PATH")]
	#[clap(short = 'C', long)]
	#[clap(default_value = ".")]
	repo: PathBuf,

	/// The repository to operate on.
	#[clap(value_name = "REMOTE")]
	remote: String,

	/// The refs to fetch.
	#[clap(trailing_var_arg = true)]
	#[clap(required = true)]
	refspec: Vec<String>,
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

	match options.command {
		Command::Clone(command) => clone(command),
		Command::Fetch(command) => fetch(command),
		Command::Push(command) => push(command),
	}
}

fn clone(command: CloneCommand) -> Result<(), ()> {
	let local_path = command.local_path.as_deref()
		.unwrap_or_else(|| Path::new(repo_name_from_url(&command.repo)));

	log::info!("Cloning {} into {}", command.repo, local_path.display());

	let auth = auth_git2::GitAuthenticator::default();
	auth.clone_repo(&command.repo, local_path)
		.map_err(|e| log::error!("Failed to clone {}: {}", command.repo, e))?;
	Ok(())
}

fn fetch(command: FetchCommand) -> Result<(), ()> {
	let repo = git2::Repository::open(&command.repo)
		.map_err(|e| log::error!("Failed to open git repo at {}: {e}", command.repo.display()))?;

	let refspecs: Vec<_> = command.refspec.iter().map(|x| x.as_str()).collect();

	let auth = auth_git2::GitAuthenticator::default();
	let mut remote = repo.find_remote(&command.remote)
		.map_err(|e| log::error!("Failed to find remote {:?}: {e}", command.remote))?;
	auth.fetch(&repo, &mut remote, &refspecs, None)
		.map_err(|e| log::error!("Failed to fetch from remote {:?}: {e}", command.remote))?;
	Ok(())
}

fn push(command: PushCommand) -> Result<(), ()> {
	let repo = git2::Repository::open(&command.repo)
		.map_err(|e| log::error!("Failed to open git repo at {}: {e}", command.repo.display()))?;

	log::info!("Fetching {:?} from remote {:?}", command.refspec, command.remote);
	let refspecs: Vec<_> = command.refspec.iter().map(|x| x.as_str()).collect();

	let auth = auth_git2::GitAuthenticator::default();
	let mut remote = repo.find_remote(&command.remote)
		.map_err(|e| log::error!("Failed to find remote {:?}: {e}", command.remote))?;
	auth.push(&repo, &mut remote, &refspecs,)
		.map_err(|e| log::error!("Failed to push to remote {:?}: {e}", command.remote))?;
	Ok(())
}

fn repo_name_from_url(url: &str) -> &str {
	url.rsplit_once('/')
		.map(|(_head, tail)| tail)
		.unwrap_or(url)
}
