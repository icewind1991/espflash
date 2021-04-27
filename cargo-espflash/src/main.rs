use std::ffi::OsString;
use std::fs::read;
use std::path::PathBuf;
use std::process::{exit, Command, ExitStatus, Stdio};

use cargo_project::{Artifact, Profile, Project};
use color_eyre::{eyre::WrapErr, Report, Result};
use espflash::{Chip, Config, Flasher};
use pico_args::Arguments;
use serial::{BaudRate, SerialPort};

fn main() -> Result<()> {
    let args = parse_args().expect("Unable to parse command-line arguments");
    let config = Config::load();

    if args.help || (args.serial.is_none() && config.connection.serial.is_none()) {
        return usage();
    }

    #[allow(clippy::or_fun_call)]
    let tool = args
        .build_tool
        .as_deref()
        .or(config.build.tool.as_deref())
        .or(Some("xbuild"));

    let tool = match tool {
        Some("xargo") | Some("cargo") | Some("xbuild") => tool.unwrap(),
        Some(_) => {
            eprintln!("Only 'xargo', 'cargo' and 'xbuild' are valid build types.");
            return Ok(());
        }
        None => return usage(),
    };

    let port = args.serial.or(config.connection.serial).unwrap();

    let speed = args.speed.map(|v| BaudRate::from_speed(v as usize));

    let chip = match args.chip.as_ref() {
        Some(chip) => chip.parse()?,
        None => chip_detect(&port).wrap_err("Unable to detect chip type, ensure your device is connected or manually specify the chip")?
    };

    let target = chip.target();

    // Since the application exits without flashing the device when '--board-info'
    // is passed, we will not waste time building if said flag was set.
    if !args.board_info {
        let status = build(args.release, &args.example, &args.features, tool, target);
        if !status.success() {
            exit_with_process_status(status)
        }
    }

    let mut serial =
        serial::open(&port).wrap_err_with(|| format!("Failed to open serial port {}", port))?;
    serial.reconfigure(&|settings| {
        settings.set_baud_rate(BaudRate::Baud115200)?;
        Ok(())
    })?;

    let mut flasher = Flasher::connect(serial, speed)?;
    if args.board_info {
        return board_info(&flasher);
    }

    let path = get_artifact_path(target, args.release, &args.example)
        .expect("Could not find the build artifact path");
    let elf_data = read(&path)?;

    if args.ram {
        flasher.load_elf_to_ram(&elf_data)?;
    } else {
        flasher.load_elf_to_flash(&elf_data)?;
    }

    Ok(())
}

#[derive(Debug)]
struct AppArgs {
    help: bool,
    board_info: bool,
    ram: bool,
    release: bool,
    example: Option<String>,
    features: Option<String>,
    chip: Option<String>,
    build_tool: Option<String>,
    speed: Option<u32>,
    serial: Option<String>,
}

#[allow(clippy::unnecessary_wraps)]
fn usage() -> Result<()> {
    let usage = "Usage: cargo espflash \
      [--board-info] \
      [--ram] \
      [--release] \
      [--example EXAMPLE] \
      [--tool {{cargo,xargo,xbuild}}] \
      [--chip {{esp32,esp8266,esp32c3}}] \
      [--speed BAUD] \
      <serial>";

    println!("{}", usage);

    Ok(())
}

#[allow(clippy::unnecessary_wraps)]
fn board_info(flasher: &Flasher) -> Result<()> {
    println!("Chip type:  {:?}", flasher.chip());
    println!("Flash size: {:?}", flasher.flash_size());

    Ok(())
}

fn parse_args() -> Result<AppArgs> {
    // Skip the command and subcommand (ie. 'cargo espflash') and convert the
    // remaining arguments to the expected type.
    let args = std::env::args().skip(2).map(OsString::from).collect();

    let mut args = Arguments::from_vec(args);

    let app_args = AppArgs {
        help: args.contains("--help"),
        board_info: args.contains("--board-info"),
        ram: args.contains("--ram"),
        release: args.contains("--release"),
        example: args.opt_value_from_str("--example")?,
        features: args.opt_value_from_str("--features")?,
        chip: args.opt_value_from_str("--chip")?,
        speed: args.opt_value_from_str("--speed")?,
        build_tool: args.opt_value_from_str("--tool")?,
        serial: args.opt_free_from_str()?,
    };

    Ok(app_args)
}

fn get_artifact_path(target: &str, release: bool, example: &Option<String>) -> Result<PathBuf> {
    let project = Project::query(".").unwrap();

    let artifact = match example {
        Some(example) => Artifact::Example(example.as_str()),
        None => Artifact::Bin(project.name()),
    };

    let profile = if release {
        Profile::Release
    } else {
        Profile::Dev
    };

    let host = "x86_64-unknown-linux-gnu";
    project
        .path(artifact, profile, Some(target), host)
        .map_err(Report::msg)
}

fn build(
    release: bool,
    example: &Option<String>,
    features: &Option<String>,
    tool: &str,
    target: &str,
) -> ExitStatus {
    let mut args: Vec<String> = vec![];

    if release {
        args.push("--release".to_string());
    }

    match example {
        Some(example) => {
            args.push("--example".to_string());
            args.push(example.to_string());
        }
        None => {}
    }

    match features {
        Some(features) => {
            args.push("--features".to_string());
            args.push(features.to_string());
        }
        None => {}
    }

    let mut command = match tool {
        "cargo" | "xbuild" => Command::new("cargo"),
        "xargo" => Command::new("xargo"),
        _ => unreachable!(),
    };

    let command = match tool {
        "xargo" | "cargo" => command.arg("build"),
        "xbuild" => command.arg("xbuild"),
        _ => unreachable!(),
    };

    if let "cargo" = tool {
        args.push("-Z".to_string());
        args.push("build-std".to_string());
    };

    args.push("--target".to_string());
    args.push(target.to_string());

    command
        .args(args)
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .spawn()
        .unwrap()
        .wait()
        .unwrap()
}

fn chip_detect(port: &str) -> Result<Chip> {
    let mut serial =
        serial::open(port).wrap_err_with(|| format!("Failed to open serial port {}", port))?;
    serial.reconfigure(&|settings| {
        settings.set_baud_rate(BaudRate::Baud115200)?;

        Ok(())
    })?;
    let flasher = Flasher::connect(serial, None)?;

    Ok(flasher.chip())
}

#[cfg(unix)]
fn exit_with_process_status(status: ExitStatus) -> ! {
    use std::os::unix::process::ExitStatusExt;
    let code = status.code().or_else(|| status.signal()).unwrap_or(1);

    exit(code)
}

#[cfg(not(unix))]
fn exit_with_process_status(status: ExitStatus) -> ! {
    let code = status.code().unwrap_or(1);

    exit(code)
}
