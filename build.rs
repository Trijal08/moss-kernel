use time::OffsetDateTime;
use time::macros::format_description;

fn main() {
    let out_dir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let linker_script = match std::env::var("CARGO_CFG_TARGET_ARCH") {
        Ok(arch) if arch == "aarch64" => {
            let src = "./src/arch/arm64/boot/linker.ld";
            let dst = out_dir.join("linker_preprocessed.ld");
            // Run CPP: -E (preprocess), -P (no line markers), -x c (treat as C)
            std::process::Command::new("cc")
                .args(["-E", "-P", "-x", "c", src, "-o"])
                .arg(&dst)
                .status().unwrap();
            dst
        },
        Ok(arch) => {
            println!("Unsupported arch: {arch}");
            std::process::exit(1);
        }
        Err(_) => unreachable!("Cargo should always set the arch"),
    };

    println!("cargo::rerun-if-changed=./src/arch/arm64/boot/linker.ld");
    println!("cargo::rerun-if-changed=./src/arch/arm64/boot/linux-kernel-image-header-vars.h");
    println!("cargo::rustc-link-arg=-T{}", linker_script.display());

    // Set an environment variable with the date and time of the build
    let now = OffsetDateTime::now_utc();
    let format = format_description!(
        "[weekday repr:short] [month repr:short] [day] [hour]:[minute]:[second] UTC [year]"
    );
    let timestamp = now.format(&format).unwrap();
    #[cfg(feature = "smp")]
    println!("cargo:rustc-env=MOSS_VERSION=#1 Moss SMP {timestamp}");
    #[cfg(not(feature = "smp"))]
    println!("cargo:rustc-env=MOSS_VERSION=#1 Moss {timestamp}");
}
