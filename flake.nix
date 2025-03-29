{
  description = "A Nix Flake for building and flashing Rust applications for Xtensa (ESP32)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
    flake-utils.url = "github:numtide/flake-utils";
    esp-dev.url = "github:mirrexagon/nixpkgs-esp-dev"; # For ESP-IDF packages
  };

  outputs = {
    self,
    nixpkgs,
    flake-utils,
    esp-dev,
  }:
    flake-utils.lib.eachDefaultSystem (system: let
      pkgs = import nixpkgs {
        inherit system;
        overlays = [esp-dev.overlays.default]; # Include ESP-IDF packages
      };
    in {
      devShells.default = pkgs.mkShell {
        name = "xtensa-esp32-dev-shell";
        buildInputs = with pkgs; [
          rustup # Add rustup for espup
          cargo # Base Cargo (will be overridden by rustup)
          cargo-espflash # For building and flashing
          espup # For managing the toolchain
          esp-idf-full # ESP-IDF for Xtensa
          python3 # Required by espup/esp-idf
        ];

        shellHook = ''
          # Use a persistent HOME directory for rustup and cargo
          export RUSTUP_HOME="$PWD/.rustup"
          export CARGO_HOME="$PWD/.cargo"
          mkdir -p "$RUSTUP_HOME" "$CARGO_HOME"

          # Update PATH to include Cargo binaries
          export PATH="$CARGO_HOME/bin:$PATH"

          # Install the Xtensa toolchain via espup if not already present
          if [ ! -f rust-toolchain.env ]; then
            echo "Installing Xtensa Rust toolchain via espup..."
            ${pkgs.espup}/bin/espup install --export-file rust-toolchain.env
          fi

          # Source the toolchain environment
          source rust-toolchain.env
          rustup default esp

          echo "Xtensa ESP32 development environment ready!"
          echo "Rust version: $(rustc --version)"
          echo "Use 'cargo build --target xtensa-esp32-none-elf' to build."
          echo "Use 'cargo espflash flash' to flash your binary."
        '';
      };

      packages.default = pkgs.rustPlatform.buildRustPackage {
        pname = "xtensa-example";
        version = "0.1.0";
        src = pkgs.lib.cleanSource ./.;
        cargoLock = {
          lockFile = ./Cargo.lock; # Assumes you have a Cargo.lock
        };
        nativeBuildInputs = with pkgs; [cargo-espflash rustup];
        buildPhase = ''
          export RUSTUP_HOME="$PWD/.rustup"
          export CARGO_HOME="$PWD/.cargo"
          mkdir -p "$RUSTUP_HOME" "$CARGO_HOME"
          ${pkgs.espup}/bin/espup install --export-file rust-toolchain.env
          source rust-toolchain.env
          cargo build --release --target xtensa-esp32-none-elf
        '';
        installPhase = ''
          mkdir -p $out/bin
          cp target/xtensa-esp32-none-elf/release/xtensa-example $out/bin/
        '';
      };

      apps.default = {
        type = "app";
        program = "${pkgs.writeShellScriptBin "flash" ''
          export RUSTUP_HOME="$PWD/.rustup"
          export CARGO_HOME="$PWD/.cargo"
          mkdir -p "$RUSTUP_HOME" "$CARGO_HOME"
          ${pkgs.espup}/bin/espup install --export-file rust-toolchain.env
          source rust-toolchain.env
          ${pkgs.cargo-espflash}/bin/cargo-espflash flash --release --target xtensa-esp32-none-elf
        ''}/bin/flash";
      };
    });
}
