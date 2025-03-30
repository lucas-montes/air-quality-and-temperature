{
  description = "A Nix Flake for building and flashing Rust applications for Xtensa (ESP32)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = {
    self,
    nixpkgs,
    flake-utils,
  }:
    flake-utils.lib.eachDefaultSystem (system: let
      pkgs = import nixpkgs {
        inherit system;
      };
    in {
      devShells.default = pkgs.mkShell {
        name = "xtensa-esp32-dev-shell";
        buildInputs = with pkgs; [
          rustup # Add rustup for espup
          cargo # Base Cargo (will be overridden by rustup)
          cargo-espflash # For building and flashing
          espup # For managing the toolchain
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
        '';
      };
    });
}
