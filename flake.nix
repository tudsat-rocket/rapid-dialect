{
  description = "mavlink rapid-dialect";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs = {
        nixpkgs.follows = "nixpkgs";
      };
    };
  };
  outputs =
    {
      rust-overlay,
      nixpkgs,
      flake-utils,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system}.extend (import rust-overlay);
        rustToolchain = pkgs.rust-bin.fromRustupToolchainFile ./rust-toolchain.toml;

        buildInputs =
          with pkgs;
          [

          ]
          ++ [ rustToolchain ];

      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = buildInputs ++ [ pkgs.just ];
        };

        formatter = pkgs.nixfmt-rfc-style;
      }
    );
}
